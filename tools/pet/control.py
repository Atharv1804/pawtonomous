"""Pet behaviours as a pure state machine: perception in -> velocity / speech / expression out.

No hardware, no I/O, no ROS: brain.py feeds it detections + ultrasonic ranges ~10 times a
second and executes what it returns. That keeps the fast loop testable (test/test_pet.py).

Conventions: bearing_deg > 0 = target is to the RIGHT of the camera centre.
angular > 0 = turn LEFT (counter-clockwise, ROS convention). linear >= 0 only (no reverse).

Modes
  idle      stand still (the face still looks at people)
  look_at   turn in place to face a target; search by spinning if it is not visible
  follow    keep facing the target and stay ~FOLLOW_DIST_M away until stopped / timeout
  go_to     like follow, but finish once close (ARRIVE_DIST_M)
  find      spin 360 deg looking; if not found, explore for a while; repeat; then go_to
  wander    explore autonomously (the obstacle-aware 'explore' behaviour does the driving)
  curious   camera-guided exploring: brain.py's curious loop asks a small vision model where to go,
            drives in short bursts, turns away from obstacles, stops to comment on interesting things
  follow_straight  test mode: drive straight at the target while it is ahead and >1 m away; never
            turn (the right wheels cannot reverse, so turns are hard); stop when close, lost or off-centre
"""

import math
from dataclasses import dataclass, field

TURN_GAIN = 1.6            # rad/s per rad of bearing error
MAX_TURN = 1.0             # rad/s while tracking
SEARCH_TURN = 0.6          # rad/s while spinning to look around
DEADBAND_DEG = 4.0         # close enough to centred
DRIVE_ALIGN_DEG = 20.0     # only drive forward when the target is within this angle
FOLLOW_DIST_M = 1.0
ARRIVE_DIST_M = 0.8
STOP_CENTER_M = 0.55       # never drive forward when the centre ultrasonic is closer than this
MAX_LINEAR = 0.25
LINEAR_GAIN = 0.4          # m/s per metre of distance error
LOST_HOLD_S = 1.5          # target briefly hidden: hold still before searching
SPIN_FULL_RAD = 2 * math.pi
EXPLORE_CHUNK_S = 20.0

DEFAULT_TIMEOUT = {"look_at": 30.0, "follow": 300.0, "go_to": 120.0, "find": 180.0, "wander": 60.0,
                   "follow_straight": 120.0, "curious": 300.0}
ANY_TARGETS = ("", "any", "anyone", "someone", "person", "me", "somebody")


@dataclass
class Detection:
    name: str              # "unknown" for an unrecognised face / person
    kind: str              # "face" or "person"
    bearing_deg: float
    distance_m: float = None   # rough estimate from box size (None if unknown)
    score: float = 1.0


@dataclass
class Action:
    linear: float = 0.0
    angular: float = 0.0
    explore: bool = False      # True = run the built-in obstacle-aware explore behaviour
    say: str = None
    expression: str = None
    event: dict = None
    finished: bool = False


@dataclass
class Command:
    mode: str = "idle"
    target: str = ""
    timeout: float = 0.0
    started: float = 0.0
    # internal
    phase: str = ""
    phase_started: float = 0.0
    spun_rad: float = 0.0
    last_seen: float = None
    last_bearing: float = 0.0
    announced: bool = False
    extra: dict = field(default_factory=dict)


def is_any(target):
    return (target or "").strip().lower() in ANY_TARGETS


def pick_target(dets, target):
    """Best detection for `target`: a name (case-insensitive) or 'any' (prefer known faces, then nearest)."""
    if not dets:
        return None
    if is_any(target):
        known = [d for d in dets if d.name != "unknown"]
        pool = known or dets
        return min(pool, key=lambda d: (d.distance_m if d.distance_m is not None else 9.0, abs(d.bearing_deg)))
    t = target.strip().lower()
    named = [d for d in dets if d.name.lower() == t]
    return max(named, key=lambda d: d.score) if named else None


def steer(bearing_deg):
    if abs(bearing_deg) < DEADBAND_DEG:
        return 0.0
    w = -TURN_GAIN * math.radians(bearing_deg)
    return max(-MAX_TURN, min(MAX_TURN, w))


def approach_speed(det, center_m, stop_at):
    """Forward speed toward a centred target, never closer than stop_at (or STOP_CENTER_M by sonar)."""
    if abs(det.bearing_deg) > DRIVE_ALIGN_DEG:
        return 0.0
    if center_m is not None and center_m < STOP_CENTER_M:
        return 0.0
    dists = [d for d in (det.distance_m, center_m) if d is not None]
    if not dists:
        return 0.1                      # unknown distance: creep
    err = min(dists) - stop_at
    return max(0.0, min(MAX_LINEAR, LINEAR_GAIN * err)) if err > 0.05 else 0.0


def close_enough(det, center_m):
    """Arrived at the person. The sonar alone is not enough: something closer (a chair) may be
    between us, so it only counts when the camera also says the person is near."""
    if det.distance_m is None:
        return center_m is not None and center_m <= ARRIVE_DIST_M
    if det.distance_m <= ARRIVE_DIST_M:
        return True
    return center_m is not None and center_m <= ARRIVE_DIST_M and det.distance_m <= 1.5


class Controller:
    def __init__(self):
        self.cmd = Command()

    def set(self, mode, target="", timeout=None, now=0.0):
        if mode not in ("idle",) + tuple(DEFAULT_TIMEOUT):
            raise ValueError("unknown mode %r" % mode)
        self.cmd = Command(mode=mode, target=target or "", started=now, phase_started=now,
                           timeout=float(timeout or DEFAULT_TIMEOUT.get(mode, 0.0)),
                           phase="spin" if mode == "find" else "")
        return self.cmd

    def stop(self, now=0.0):
        self.set("idle", now=now)

    def _finish(self, now, result, say=None, expression=None, **data):
        c = self.cmd
        ev = dict(mode=c.mode, target=c.target or "anyone", result=result,
                  seconds=round(now - c.started, 1), **data)
        self.stop(now)
        return Action(finished=True, event=ev, say=say, expression=expression)

    def _search(self, c, now, dt):
        """Target not visible: hold briefly, then spin toward where it was last seen."""
        if c.last_seen is not None and now - c.last_seen < LOST_HOLD_S:
            return Action()
        direction = -1.0 if c.last_bearing > 0 else 1.0     # last seen on the right -> turn right
        c.spun_rad += SEARCH_TURN * dt
        return Action(angular=direction * SEARCH_TURN, expression="searching")

    def step(self, dets, ranges, now, dt):
        """One control tick. dets: list[Detection]; ranges: {'left','center','right': metres|None}."""
        c = self.cmd
        if c.mode == "idle":
            return Action()
        if c.timeout and now - c.started > c.timeout:
            if c.mode == "wander":
                return self._finish(now, "done")
            return self._finish(now, "timeout", say=("I couldn't find %s." % c.target)
                                if c.mode == "find" and not is_any(c.target) else None, expression="sad")
        center = (ranges or {}).get("center")
        det = pick_target(dets, c.target)
        if det is not None:
            c.last_seen, c.last_bearing = now, det.bearing_deg
            # "follow me" / "any": lock onto the first KNOWN person picked, so the robot does not
            # switch to someone else who walks past mid-follow.
            if is_any(c.target) and det.name != "unknown" and c.mode in ("follow", "go_to", "look_at",
                                                                           "follow_straight"):
                c.target = det.name

        if c.mode == "wander":
            return Action(explore=True, expression="curious")
        if c.mode == "curious":                              # brain.py drives this mode (vision model)
            return Action()

        if c.mode == "find":
            if det is not None:
                name = det.name if det.name != "unknown" else "someone"
                self.cmd = Command(mode="go_to", target=c.target, started=now, timeout=DEFAULT_TIMEOUT["go_to"],
                                   last_seen=now, last_bearing=det.bearing_deg, announced=True)
                return Action(angular=steer(det.bearing_deg), expression="happy",
                              say="Found you, %s!" % name if name != "someone" else "I see someone!",
                              event=dict(mode="find", target=c.target or "anyone", result="found",
                                         seconds=round(now - c.started, 1), bearing_deg=round(det.bearing_deg, 1)))
            if c.phase == "spin":
                c.spun_rad += SEARCH_TURN * dt
                if c.spun_rad >= SPIN_FULL_RAD:
                    c.phase, c.phase_started, c.spun_rad = "explore", now, 0.0
                return Action(angular=SEARCH_TURN, expression="searching")
            if now - c.phase_started > EXPLORE_CHUNK_S:          # explored a while: look around again
                c.phase, c.phase_started = "spin", now
            return Action(explore=True, expression="searching")

        if c.mode == "follow_straight":                      # no turning at all
            if det is None:
                return Action(expression="searching")
            return Action(linear=approach_speed(det, center, FOLLOW_DIST_M), expression="focused")

        if det is None:                                      # look_at / follow / go_to, target hidden
            return self._search(c, now, dt)

        w = steer(det.bearing_deg)
        if c.mode == "look_at":
            if w == 0.0:
                return self._finish(now, "ok", expression="happy", bearing_deg=round(det.bearing_deg, 1))
            return Action(angular=w, expression="focused")
        if c.mode == "go_to" and close_enough(det, center):
            name = det.name if det.name != "unknown" else ""
            return self._finish(now, "arrived", say=("Here I am, %s!" % name) if name else "Here I am!",
                                expression="happy")
        stop_at = ARRIVE_DIST_M if c.mode == "go_to" else FOLLOW_DIST_M
        return Action(linear=approach_speed(det, center, stop_at), angular=w, expression="focused")

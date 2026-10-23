"""Reactive behaviours that decide with the 3 ultrasonic ranges. Pure Python.

The camera is mounted high and cannot see the floor near the robot, so the
ultrasonics are the robot's only close-range "eyes". These behaviours:
  forward  - drive roughly N metres, stop early if something is in the way
  turn     - rotate roughly N degrees (+ = left / counter-clockwise)
  scan     - spin 360 deg in steps, measure distance ahead at each heading
  explore  - wander: steer away from near walls, turn toward the open side,
             back up and escape when stuck

There are no wheel encoders yet, so distance and angle are open-loop (time x
speed). Calibrate speed_gain / turn_gain once the motors run (see README).
"""

import math
import random
from dataclasses import dataclass, field, fields

FAR = 4.0  # metres, used when a sensor reports "no echo"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


@dataclass
class Ranges:
    left: float = FAR
    center: float = FAR
    right: float = FAR
    fresh: bool = True  # False if any sensor is stale/faulty

    @classmethod
    def from_optional(cls, left, center, right, fresh=True):
        f = lambda d: FAR if d is None else d
        return cls(f(left), f(center), f(right), fresh)


@dataclass
class NavConfig:
    cruise_speed: float = 0.25   # m/s commanded
    turn_speed: float = 1.2      # rad/s commanded
    speed_gain: float = 1.0      # real m/s per commanded m/s   (calibrate)
    turn_gain: float = 1.0       # real rad/s per commanded rad/s (calibrate)
    avoid_center: float = 0.45   # m: explore turns away under this
    avoid_side: float = 0.28
    stop_center: float = 0.30    # m: forward() gives up under this
    stop_side: float = 0.20
    steer_zone: float = 0.80     # m: side obstacles closer than this steer us
    steer_gain: float = 1.2      # rad/s per metre of left/right difference
    backup_speed: float = 0.15
    backup_time: float = 0.6
    stuck_count: int = 3         # this many avoids ...
    stuck_window: float = 8.0    # ... within this many seconds = stuck
    max_forward_m: float = 2.0
    scan_step_deg: float = 30.0
    scan_settle: float = 0.4


@dataclass
class Out:
    lin: float = 0.0
    ang: float = 0.0
    done: bool = False
    result: str = "ok"
    data: dict = field(default_factory=dict)


def _blocked(cfg, r):
    return r.center < cfg.stop_center or r.left < cfg.stop_side or r.right < cfg.stop_side


def _median(xs):
    s = sorted(xs)
    return s[len(s) // 2] if s else FAR


class Forward:
    name = "forward"

    def __init__(self, cfg, distance):
        self.cfg = cfg
        self.speed = cfg.cruise_speed
        self.duration = distance / (self.speed * cfg.speed_gain)
        self.t = 0.0

    def _travelled(self):
        return round(self.t * self.speed * self.cfg.speed_gain, 3)

    def tick(self, dt, r):
        if not r.fresh:
            return Out(done=True, result="sensors_unavailable")
        if _blocked(self.cfg, r):
            return Out(done=True, result="blocked", data={"travelled_m": self._travelled()})
        self.t += dt
        if self.t >= self.duration:
            return Out(done=True, result="ok", data={"travelled_m": self._travelled()})
        return Out(lin=self.speed)


class Turn:
    name = "turn"

    def __init__(self, cfg, degrees):
        self.cfg = cfg
        self.degrees = degrees
        self.dir = 1.0 if degrees > 0 else -1.0
        self.duration = math.radians(abs(degrees)) / (cfg.turn_speed * cfg.turn_gain)
        self.t = 0.0

    def tick(self, dt, r):
        self.t += dt
        if self.t >= self.duration:
            return Out(done=True, result="ok", data={"degrees": self.degrees})
        return Out(ang=self.dir * self.cfg.turn_speed)


class Scan:
    name = "scan"

    def __init__(self, cfg):
        self.cfg = cfg
        self.n = max(1, int(round(360.0 / cfg.scan_step_deg)))
        self.i = 0
        self.state = "settle"
        self.timer = cfg.scan_settle
        self.samples = []
        self.vals = []
        self.turn_dur = math.radians(cfg.scan_step_deg) / (cfg.turn_speed * cfg.turn_gain)

    def tick(self, dt, r):
        if not r.fresh:
            return Out(done=True, result="sensors_unavailable")
        if self.state == "settle":
            self.samples.append(r.center)
            self.timer -= dt
            if self.timer <= 0:
                self.vals.append(_median(self.samples))
                self.samples = []
                self.state = "turn"
                self.timer = self.turn_dur
            return Out()
        self.timer -= dt
        if self.timer <= 0:
            self.i += 1
            if self.i >= self.n:
                return Out(done=True, result="ok", data=self._result())
            self.state = "settle"
            self.timer = self.cfg.scan_settle
            return Out()
        return Out(ang=self.cfg.turn_speed)

    def _result(self):
        step = self.cfg.scan_step_deg
        best = max(range(len(self.vals)), key=lambda k: self.vals[k])
        heading = best * step
        if heading > 180:
            heading -= 360
        return {
            "step_deg": step,
            "note": "index 0 = straight ahead, then counter-clockwise (left); approximate, no encoders",
            "ranges_m": [round(v, 2) for v in self.vals],
            "best_heading_deg": heading,
            "best_range_m": round(self.vals[best], 2),
        }


class Explore:
    name = "explore"

    def __init__(self, cfg, duration, rng=None):
        self.cfg = cfg
        self.duration = duration
        self.rng = rng or random.Random()
        self.t = 0.0
        self.state = "cruise"      # cruise | backup | turn
        self.timer = 0.0
        self.dir = 1
        self.after_backup = (60, 120)
        self.avoid_times = []
        self.avoids = 0

    def _register_avoid(self):
        self.avoids += 1
        self.avoid_times.append(self.t)
        self.avoid_times = [x for x in self.avoid_times if self.t - x <= self.cfg.stuck_window]
        return len(self.avoid_times) >= self.cfg.stuck_count

    def _choose_dir(self, r):
        if abs(r.left - r.right) < 0.05:
            return self.rng.choice((-1, 1))
        return 1 if r.left > r.right else -1   # turn toward the more open side

    def _begin_turn(self, lo, hi):
        deg = self.rng.uniform(lo, hi)
        self.timer = math.radians(deg) / (self.cfg.turn_speed * self.cfg.turn_gain)
        self.state = "turn"

    def tick(self, dt, r):
        c = self.cfg
        if not r.fresh:
            return Out(done=True, result="sensors_unavailable", data={"avoids": self.avoids})
        self.t += dt
        if self.t >= self.duration:
            return Out(done=True, result="ok", data={"avoids": self.avoids})

        if self.state == "backup":
            self.timer -= dt
            if self.timer <= 0:
                self._begin_turn(*self.after_backup)
            return Out(lin=-c.backup_speed)

        if self.state == "turn":
            self.timer -= dt
            if self.timer <= 0:
                self.state = "cruise"
            return Out(ang=self.dir * c.turn_speed)

        # ---- cruise ----
        if r.center < c.avoid_center or r.left < c.avoid_side or r.right < c.avoid_side:
            stuck = self._register_avoid()
            self.dir = self._choose_dir(r)
            if stuck:
                self.avoid_times = []
                self.state, self.timer, self.after_backup = "backup", c.backup_time, (150, 180)
            elif r.center < c.avoid_center * 0.6:
                self.state, self.timer, self.after_backup = "backup", c.backup_time, (60, 120)
            else:
                self._begin_turn(45, 110)
            return Out()

        speed = c.cruise_speed * clamp((r.center - c.avoid_center) / (1.0 - c.avoid_center), 0.4, 1.0)
        dl, dr = min(r.left, c.steer_zone), min(r.right, c.steer_zone)
        ang = c.steer_gain * (dl - dr) if min(dl, dr) < c.steer_zone else 0.0
        return Out(lin=speed, ang=ang)


def _num(cmd, key, default=None):
    v = cmd.get(key, default)
    if v is None:
        raise ValueError("%s is required" % key)
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ValueError("%s must be a number" % key)
    if math.isnan(v):
        raise ValueError("%s must be a number" % key)
    return v


class NavController:
    """Runs one behaviour at a time. tick() returns (lin, ang, events);
    lin/ang are None while idle (so the node publishes nothing)."""

    def __init__(self, cfg=None, rng=None, max_runtime=330.0):
        self.cfg = cfg or NavConfig()
        self.rng = rng or random.Random()
        self.max_runtime = max_runtime
        self.active = None
        self.cid = ""
        self.name = None
        self.runtime = 0.0

    def _event(self, result, data=None):
        return {"id": self.cid, "behavior": self.name, "result": result, "data": data or {}}

    def _build(self, cmd):
        name = cmd.get("behavior")
        c = self.cfg
        if name == "forward":
            d = _num(cmd, "distance", 0.5)
            if d <= 0:
                raise ValueError("distance must be > 0 (reversing is not supported: no rear sensors)")
            return Forward(c, clamp(d, 0.05, c.max_forward_m))
        if name == "turn":
            deg = clamp(_num(cmd, "degrees"), -360.0, 360.0)
            if deg == 0:
                raise ValueError("degrees must not be 0")
            return Turn(c, deg)
        if name == "scan":
            return Scan(c)
        if name == "explore":
            return Explore(c, clamp(_num(cmd, "seconds", 30.0), 1.0, 300.0), self.rng)
        if name == "stop":
            return None
        raise ValueError("unknown behavior %r" % (name,))

    def start(self, cmd):
        """Begin a new behaviour (preempting any running one). Returns events."""
        events = []
        if self.active is not None:
            events.append(self._event("preempted"))
            self.active = None
        self.cid = str(cmd.get("id", ""))
        self.name = cmd.get("behavior")
        try:
            beh = self._build(cmd)
        except ValueError as e:
            events.append(self._event("error", {"error": str(e)}))
            return events
        if beh is None:  # stop
            events.append(self._event("ok"))
            return events
        self.active = beh
        self.runtime = 0.0
        return events

    def tick(self, dt, ranges):
        if self.active is None:
            return None, None, []
        self.runtime += dt
        if self.runtime > self.max_runtime:
            ev = self._event("timeout")
            self.active = None
            return 0.0, 0.0, [ev]
        out = self.active.tick(dt, ranges)
        if out.done:
            ev = self._event(out.result, out.data)
            self.active = None
            return 0.0, 0.0, [ev]
        return out.lin, out.ang, []


def config_from_dict(d):
    """Build a NavConfig from a dict, ignoring unknown keys."""
    names = {f.name for f in fields(NavConfig)}
    return NavConfig(**{k: v for k, v in d.items() if k in names})

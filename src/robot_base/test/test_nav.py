import math
import random

import pytest

from robot_base.nav import FAR, NavConfig, NavController, Ranges
from robot_base.sensor_health import SensorHealth

DT = 0.05
CLEAR = Ranges(FAR, FAR, FAR, True)


def run(ctrl, ranges_fn, max_s=60.0):
    """Tick the controller until it finishes. Returns (events, trace)."""
    t, trace, events = 0.0, [], []
    while t < max_s:
        lin, ang, evs = ctrl.tick(DT, ranges_fn(t))
        events += evs
        trace.append((lin, ang))
        if evs and ctrl.active is None:
            break
        t += DT
    return events, trace


def ctl(**kw):
    return NavController(NavConfig(**kw), rng=random.Random(1))


# ---------------- basics ----------------
def test_idle_returns_none():
    c = ctl()
    assert c.tick(DT, CLEAR) == (None, None, [])


def test_unknown_behavior_is_error_event():
    c = ctl()
    evs = c.start({"id": "a", "behavior": "dance"})
    assert evs[0]["result"] == "error" and c.active is None


def test_stop_command_acks_and_clears():
    c = ctl()
    c.start({"id": "a", "behavior": "turn", "degrees": 90})
    evs = c.start({"id": "b", "behavior": "stop"})
    assert [e["result"] for e in evs] == ["preempted", "ok"] and c.active is None


def test_new_command_preempts():
    c = ctl()
    c.start({"id": "a", "behavior": "explore", "seconds": 30})
    evs = c.start({"id": "b", "behavior": "turn", "degrees": 10})
    assert evs[0]["result"] == "preempted" and evs[0]["id"] == "a"


def test_runtime_timeout_guard():
    c = NavController(NavConfig(), rng=random.Random(1), max_runtime=1.0)
    c.start({"id": "a", "behavior": "explore", "seconds": 300})
    events, _ = run(c, lambda t: CLEAR, max_s=5)
    assert events[-1]["result"] == "timeout"


# ---------------- forward ----------------
def test_forward_completes_in_expected_time():
    c = ctl(cruise_speed=0.25)
    c.start({"id": "f", "behavior": "forward", "distance": 0.5})
    events, trace = run(c, lambda t: CLEAR)
    assert events[0]["result"] == "ok"
    assert abs(len(trace) * DT - 2.0) < 0.2        # 0.5 m / 0.25 m/s
    assert all(l == 0.25 for l, a in trace[:-1])


def test_forward_stops_early_when_blocked():
    c = ctl()
    c.start({"id": "f", "behavior": "forward", "distance": 1.5})
    events, _ = run(c, lambda t: Ranges(FAR, 1.0 if t < 1.0 else 0.2, FAR, True))
    assert events[0]["result"] == "blocked"
    assert 0 < events[0]["data"]["travelled_m"] < 1.5


def test_forward_refuses_without_good_sensors():
    c = ctl()
    c.start({"id": "f", "behavior": "forward", "distance": 0.5})
    lin, ang, evs = c.tick(DT, Ranges(FAR, FAR, FAR, False))
    assert evs[0]["result"] == "sensors_unavailable" and lin == 0.0


@pytest.mark.parametrize("d", [0, -0.5, "abc"])
def test_forward_rejects_bad_distance(d):
    c = ctl()
    evs = c.start({"id": "f", "behavior": "forward", "distance": d})
    assert evs[-1]["result"] == "error"


def test_forward_distance_is_capped():
    c = ctl(max_forward_m=2.0, cruise_speed=0.25)
    c.start({"id": "f", "behavior": "forward", "distance": 50})
    events, trace = run(c, lambda t: CLEAR, max_s=30)
    assert len(trace) * DT < 8.5                   # 2 m, not 50 m


# ---------------- turn ----------------
def test_turn_left_positive_and_duration():
    c = ctl(turn_speed=1.0)
    c.start({"id": "t", "behavior": "turn", "degrees": 90})
    events, trace = run(c, lambda t: CLEAR)
    assert events[0]["result"] == "ok"
    assert trace[0][1] > 0
    assert abs(len(trace) * DT - math.pi / 2) < 0.15


def test_turn_right_negative():
    c = ctl()
    c.start({"id": "t", "behavior": "turn", "degrees": -45})
    _, trace = run(c, lambda t: CLEAR)
    assert trace[0][1] < 0


def test_turn_gain_changes_duration():
    a, b = ctl(turn_gain=1.0), ctl(turn_gain=0.5)
    for x in (a, b):
        x.start({"id": "t", "behavior": "turn", "degrees": 90})
    _, ta = run(a, lambda t: CLEAR)
    _, tb = run(b, lambda t: CLEAR)
    assert len(tb) > 1.8 * len(ta)


# ---------------- scan ----------------
def test_scan_returns_12_headings_and_best_direction():
    c = ctl()
    c.start({"id": "s", "behavior": "scan"})
    # open area is at the 4th step (90 deg left); everything else 1 m
    def rng(t):
        step_time = c.cfg.scan_settle + c.active.turn_dur if c.active else 1
        idx = int(t // step_time)
        return Ranges(FAR, 3.0 if idx == 3 else 1.0, FAR, True)
    events, _ = run(c, rng, max_s=60)
    data = events[0]["data"]
    assert events[0]["result"] == "ok" and len(data["ranges_m"]) == 12
    assert data["best_heading_deg"] == 90 and data["best_range_m"] == 3.0


# ---------------- explore ----------------
def first_motion(c, r, n=200):
    for _ in range(n):
        lin, ang, _ = c.tick(DT, r)
        if (lin, ang) != (0.0, 0.0):
            return lin, ang
    return 0.0, 0.0


def test_explore_cruises_when_clear():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 10})
    lin, ang, _ = c.tick(DT, CLEAR)
    assert lin == pytest.approx(0.25) and ang == 0


def test_explore_turns_toward_open_side_left():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 30})
    lin, ang = first_motion(c, Ranges(2.0, 0.35, 0.3, True))
    assert lin == 0.0 and ang > 0          # left is more open -> turn left


def test_explore_turns_toward_open_side_right():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 30})
    lin, ang = first_motion(c, Ranges(0.3, 0.35, 2.0, True))
    assert ang < 0


def test_explore_very_close_backs_up_first():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 30})
    c.tick(DT, Ranges(1.0, 0.1, 1.0, True))        # triggers avoid
    lin, ang, _ = c.tick(DT, Ranges(1.0, 0.1, 1.0, True))
    assert lin < 0


def test_explore_steers_away_from_near_left_wall():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 30})
    lin, ang, _ = c.tick(DT, Ranges(0.5, FAR, FAR, True))
    assert lin > 0 and ang < 0              # wall on left -> veer right


def test_explore_escapes_when_stuck():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 60})
    boxed = Ranges(0.2, 0.2, 0.2, True)
    saw_backup = False
    for _ in range(int(20 / DT)):
        lin, ang, evs = c.tick(DT, boxed)
        saw_backup |= lin < 0
        if evs:
            break
    assert saw_backup and c.active is not None


def test_explore_ends_after_duration():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 2})
    events, _ = run(c, lambda t: CLEAR, max_s=10)
    assert events[0]["result"] == "ok"


def test_explore_stops_if_sensors_go_bad():
    c = ctl()
    c.start({"id": "e", "behavior": "explore", "seconds": 60})
    c.tick(DT, CLEAR)
    lin, ang, evs = c.tick(DT, Ranges(FAR, FAR, FAR, False))
    assert evs[0]["result"] == "sensors_unavailable"


# ---------------- sensor health ----------------
def test_health_needs_data_from_all_sensors():
    h = SensorHealth()
    h.update("left", 1.0, 0.0)
    ok, why = h.status(0.1)
    assert not ok and "center" in why


def test_health_ok_and_stale():
    h = SensorHealth(timeout=0.5)
    for n in ("left", "center", "right"):
        h.update(n, 1.0, 0.0)
    assert h.status(0.3)[0]
    assert not h.status(1.0)[0]


def test_health_flags_stuck_no_echo():
    h = SensorHealth(timeout=0.5, max_noecho_s=8.0)
    t = 0.0
    while t < 9.0:
        for n in ("left", "center", "right"):
            h.update(n, None, t)             # never any echo (unpowered sensor)
        t += 0.1
    ok, why = h.status(t)
    assert not ok and "stuck" in why


def test_health_recovers_when_echo_returns():
    h = SensorHealth(max_noecho_s=2.0)
    for i in range(40):
        for n in ("left", "center", "right"):
            h.update(n, None, i * 0.1)
    assert not h.status(4.0)[0]
    for n in ("left", "center", "right"):
        h.update(n, 1.2, 4.05)
    assert h.status(4.1)[0]


def test_health_stuck_check_can_be_disabled():
    h = SensorHealth(max_noecho_s=0)
    for i in range(400):
        for n in ("left", "center", "right"):
            h.update(n, None, i * 0.1)
    assert h.status(40.0)[0]

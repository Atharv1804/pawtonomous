"""Unit tests for the pure-Python logic (no ROS needed): pytest src/robot_base/test"""

import math

import pytest

from robot_base.mixer import twist_to_wheels
from robot_base.protocol import STOP, LineBuffer, format_vel, parse_range_line, with_key
from robot_base.safety import SafetyConfig, limit

MAXLIN, MAXANG, MAXPWM, MINPWM = 0.4, 2.0, 200, 60


def wheels(lin, ang):
    return twist_to_wheels(lin, ang, MAXLIN, MAXANG, MAXPWM, MINPWM)


# ---------------- mixer ----------------
def test_stop_is_zero():
    assert wheels(0, 0) == (0, 0)


def test_full_forward():
    assert wheels(0.4, 0) == (200, 200)


def test_full_reverse():
    assert wheels(-0.4, 0) == (-200, -200)


def test_beyond_max_is_clamped():
    assert wheels(5.0, 0) == (200, 200)


def test_spin_left_in_place():
    # +angular = counter-clockwise = left turn: left wheel back, right forward
    l, r = wheels(0, 2.0)
    assert l < 0 < r and l == -r == -200


def test_spin_right_in_place():
    l, r = wheels(0, -2.0)
    assert r < 0 < l


def test_curve_left_right_wheel_faster():
    l, r = wheels(0.2, 0.5)
    assert 0 < l < r


def test_min_pwm_applied_to_small_commands():
    # smallest non-zero command starts at min_pwm, not at ~0 where motors only hum
    l, r = wheels(0.0005, 0)
    assert l == r == MINPWM
    # and the ramp above it is monotonic up to max_pwm
    assert MINPWM < wheels(0.1, 0)[0] < wheels(0.3, 0)[0] <= MAXPWM


def test_tiny_noise_is_zero():
    assert wheels(0.00001, 0) == (0, 0)


def test_mix_keeps_ratio_when_saturating():
    l, r = wheels(0.4, 1.0)  # lin=1, ang=.5 -> 0.5 / 1.5 -> scaled
    assert max(abs(l), abs(r)) <= MAXPWM
    assert r == 200 and 0 < l < r


def test_bad_limits_rejected():
    with pytest.raises(ValueError):
        twist_to_wheels(0.1, 0, 0, 2.0, 200)


# ---------------- protocol ----------------
def test_format_vel():
    assert format_vel(120, -80) == b"VEL 120 -80"


def test_keyed_packets():
    assert format_vel(120, -80, "k3y") == b"k3y VEL 120 -80"
    assert with_key(STOP, "k3y") == b"k3y STOP"
    assert with_key(STOP, "") == b"STOP"


def test_parse_ok_and_no_echo():
    assert parse_range_line("D,230,1500,-1") == [0.23, 1.5, None]


@pytest.mark.parametrize(
    "line", ["", "garbage", "D,1,2", "D,1,2,3,4", "D,a,b,c", "X,1,2,3"]
)
def test_parse_rejects_bad_lines(line):
    assert parse_range_line(line) is None


def test_parse_handles_crlf_whitespace():
    assert parse_range_line("D,100,200,300\r\n") == [0.1, 0.2, 0.3]


def test_linebuffer_splits_partial_chunks():
    b = LineBuffer()
    assert b.feed(b"D,10,2") == []
    assert b.feed(b"0,30\r\nD,1,2,3\n") == ["D,10,20,30", "D,1,2,3"]


def test_linebuffer_drops_runaway_garbage():
    b = LineBuffer()
    b.feed(b"x" * 1000)
    assert b.feed(b"D,1,2,3\n") == ["D,1,2,3"]


# ---------------- safety ----------------
CFG = SafetyConfig()


def run(lin, ang, l=None, c=None, r=None, fresh=True):
    return limit(lin, ang, l, c, r, fresh, CFG)


def test_clear_passes_through():
    assert run(0.3, 0.1) == (0.3, 0.1, "clear")


def test_no_echo_is_clear():
    assert run(0.3, 0, None, None, None)[0] == 0.3


def test_blocked_center_stops_forward_keeps_turn():
    lin, ang, why = run(0.3, 0.5, c=0.2)
    assert lin == 0.0 and ang == 0.5 and why == "blocked"


def test_blocked_stops_dead_when_no_turn():
    assert run(0.3, 0, c=0.2)[:2] == (0.0, 0)


def test_slowing_is_between_zero_and_full():
    lin, _, why = run(0.4, 0, c=0.475)  # halfway between 0.25 and 0.70
    assert why == "slowing" and math.isclose(lin, 0.2, rel_tol=1e-6)


def test_side_sensor_blocks():
    assert run(0.3, 0, l=0.1)[2] == "blocked"
    assert run(0.3, 0, r=0.1)[2] == "blocked"


def test_stale_sensors_block_forward():
    lin, _, why = run(0.3, 0, fresh=False)
    assert lin == 0.0 and why.startswith("blind")


def test_stale_sensors_allowed_if_configured():
    cfg = SafetyConfig(allow_blind_forward=True)
    assert limit(0.3, 0, None, None, None, False, cfg)[0] == 0.3


def test_reverse_is_capped_even_if_blocked_ahead():
    lin, _, why = run(-0.4, 0, c=0.05)
    assert math.isclose(lin, -0.2) and "reversing" in why


def test_pure_spin_always_allowed():
    assert run(0, 2.0, l=0.05, c=0.05, r=0.05)[:2] == (0, 2.0)

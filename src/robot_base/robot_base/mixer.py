"""Convert a ROS Twist (linear m/s, angular rad/s) into left/right wheel PWM.

Pure Python, no ROS imports, so it can be unit-tested anywhere.

Sign convention (ROS REP-103): +linear = forward, +angular = turn LEFT
(counter-clockwise). Wheel outputs: +PWM = wheel forward.
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _to_pwm(x, max_pwm, min_pwm):
    """x in [-1, 1] -> signed PWM. Any non-zero x starts at min_pwm so the
    motors don't just hum without turning at very low duty."""
    if abs(x) < 1e-3:
        return 0
    mag = min_pwm + (max_pwm - min_pwm) * abs(x)
    return int(round(mag if x > 0 else -mag))


def twist_to_wheels(linear, angular, max_linear, max_angular, max_pwm, min_pwm=0):
    """Return (left_pwm, right_pwm), each in [-max_pwm, max_pwm]."""
    if max_linear <= 0 or max_angular <= 0:
        raise ValueError("max_linear and max_angular must be > 0")
    lin = clamp(linear / max_linear, -1.0, 1.0)
    ang = clamp(angular / max_angular, -1.0, 1.0)

    left = lin - ang
    right = lin + ang

    # If a wheel would exceed 100 %, scale both so the turn ratio is kept.
    scale = max(1.0, abs(left), abs(right))
    left /= scale
    right /= scale

    return _to_pwm(left, max_pwm, min_pwm), _to_pwm(right, max_pwm, min_pwm)

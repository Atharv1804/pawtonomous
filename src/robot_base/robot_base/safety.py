"""Obstacle reflex: limits a requested velocity using the 3 ultrasonic ranges.

Pure Python, no ROS imports. This is the layer that keeps the robot from
driving into things no matter what the agent/LLM asks for.

Limits of 3 front ultrasonic sensors (be aware of these):
  * no rear sensors  -> reversing is "blind", so it is slowed
  * soft/angled surfaces can absorb or deflect the pulse (reads as 'clear')
  * stairs / drops and low objects below the beam are NOT detected
"""

from dataclasses import dataclass


@dataclass
class SafetyConfig:
    stop_center: float = 0.25   # m: forward blocked at/under this (centre)
    slow_center: float = 0.70   # m: start slowing under this
    stop_side: float = 0.18     # m: same for the left/right sensors
    slow_side: float = 0.45
    reverse_scale: float = 0.5  # reversing is blind: cap its speed
    allow_blind_forward: bool = False  # drive forward if sensor data is stale?


def _scale(d, stop, slow):
    """1.0 = free, 0.0 = blocked. d=None means 'no echo' (nothing in range)."""
    if d is None:
        return 1.0
    if d <= stop:
        return 0.0
    if d >= slow:
        return 1.0
    return (d - stop) / (slow - stop)


def limit(linear, angular, left, center, right, fresh, cfg=None):
    """Return (linear, angular, reason).

    left/center/right: distance in metres, or None for 'no echo'.
    fresh: True if all sensors reported recently.
    Turning in place is always allowed (that is how it escapes a blocked path).
    """
    cfg = cfg or SafetyConfig()

    if linear > 0:
        if not fresh and not cfg.allow_blind_forward:
            return 0.0, angular, "blind: sensor data stale"
        f = min(
            _scale(center, cfg.stop_center, cfg.slow_center),
            _scale(left, cfg.stop_side, cfg.slow_side),
            _scale(right, cfg.stop_side, cfg.slow_side),
        )
        if f <= 0.0:
            return 0.0, angular, "blocked"
        if f < 1.0:
            return linear * f, angular, "slowing"
        return linear, angular, "clear"

    if linear < 0:
        return linear * cfg.reverse_scale, angular, "reversing (blind)"

    return linear, angular, "clear"

"""safety_reflex: sits between whoever wants to drive and the motors.

  cmd_vel_raw (agent / behaviours / teleop)  ->  [safety_reflex]  ->  cmd_vel -> motor_bridge

Uses the three ultrasonic ranges to slow/stop forward motion. Also zeroes the
output if cmd_vel_raw goes quiet, and blocks forward motion if the sensor data
is stale OR a sensor looks dead (stuck on "no echo", e.g. VCC/GND not wired).
Publishes a short reason on safety/status when it changes.
"""

import math
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String

from robot_base.safety import SafetyConfig, limit
from robot_base.sensor_health import NAMES, SensorHealth


class SafetyReflex(Node):
    def __init__(self):
        super().__init__("safety_reflex")
        d = SafetyConfig()
        self.declare_parameter("stop_center", d.stop_center)
        self.declare_parameter("slow_center", d.slow_center)
        self.declare_parameter("stop_side", d.stop_side)
        self.declare_parameter("slow_side", d.slow_side)
        self.declare_parameter("reverse_scale", d.reverse_scale)
        self.declare_parameter("allow_blind_forward", d.allow_blind_forward)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("range_timeout", 0.5)
        self.declare_parameter("max_noecho_s", 8.0)
        self.declare_parameter("rate_hz", 20.0)

        p = self.get_parameter
        self.cfg = SafetyConfig(
            stop_center=float(p("stop_center").value),
            slow_center=float(p("slow_center").value),
            stop_side=float(p("stop_side").value),
            slow_side=float(p("slow_side").value),
            reverse_scale=float(p("reverse_scale").value),
            allow_blind_forward=bool(p("allow_blind_forward").value),
        )
        self.cmd_timeout = float(p("cmd_timeout").value)
        self.health = SensorHealth(
            NAMES, float(p("range_timeout").value), float(p("max_noecho_s").value)
        )

        self.raw = Twist()
        self.raw_time = None
        self.dist = {n: None for n in NAMES}   # metres, None = no echo
        self.last_reason = None

        self.create_subscription(Twist, "cmd_vel_raw", self._on_raw, 10)
        for n in NAMES:
            self.create_subscription(
                Range, "range/" + n, partial(self._on_range, n), qos_profile_sensor_data
            )
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "safety/status", 10)
        self.create_timer(1.0 / float(p("rate_hz").value), self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_raw(self, msg):
        self.raw = msg
        self.raw_time = self._now()

    def _on_range(self, name, msg):
        r = msg.range
        d = None if (math.isinf(r) or math.isnan(r) or r >= msg.max_range) else r
        self.dist[name] = d
        self.health.update(name, d, self._now())

    def _tick(self):
        out = Twist()
        reason = "idle"
        if self.raw_time is not None and (self._now() - self.raw_time) < self.cmd_timeout:
            ok, why = self.health.status(self._now())
            lin, ang, reason = limit(
                self.raw.linear.x, self.raw.angular.z,
                self.dist["left"], self.dist["center"], self.dist["right"],
                ok, self.cfg,
            )
            if reason.startswith("blind"):
                reason = "blind: " + why
            out.linear.x = float(lin)
            out.angular.z = float(ang)
        self.pub.publish(out)
        if reason != self.last_reason:
            self.last_reason = reason
            self.status_pub.publish(String(data=reason))
            self.get_logger().info("safety: " + reason)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyReflex()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""behavior_node: runs one high-level behaviour at a time (forward / turn /
scan / explore) using the ultrasonic ranges, and publishes velocity to
cmd_vel_raw (the safety_reflex still sits in front of the motors).

  behavior/command  (std_msgs/String, JSON)  e.g. {"id":"a1","behavior":"forward","distance":0.5}
  behavior/event    (std_msgs/String, JSON)  published when a behaviour ends
  behavior/state    (std_msgs/String, JSON)  2 Hz: active behaviour + sensor health
"""

import json
import math
from dataclasses import fields
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String

from robot_base.nav import NavConfig, NavController, Ranges
from robot_base.sensor_health import NAMES, SensorHealth


class BehaviorNode(Node):
    def __init__(self):
        super().__init__("behavior_node")
        # Every NavConfig field is a ROS parameter (write decimals, e.g. 1.0 not 1).
        for f in fields(NavConfig):
            self.declare_parameter(f.name, f.default)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("range_timeout", 0.5)
        self.declare_parameter("max_noecho_s", 8.0)

        cfg = NavConfig(**{f.name: self.get_parameter(f.name).value for f in fields(NavConfig)})
        self.nav = NavController(cfg)
        self.health = SensorHealth(
            NAMES,
            timeout=float(self.get_parameter("range_timeout").value),
            max_noecho_s=float(self.get_parameter("max_noecho_s").value),
        )
        self.dist = {n: None for n in NAMES}  # metres, None = no echo

        for n in NAMES:
            self.create_subscription(
                Range, "range/" + n, partial(self._on_range, n), qos_profile_sensor_data
            )
        self.create_subscription(String, "behavior/command", self._on_command, 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_raw", 10)
        self.event_pub = self.create_publisher(String, "behavior/event", 10)
        self.state_pub = self.create_publisher(String, "behavior/state", 10)

        self.last_t = self._now()
        self.last_state_t = 0.0
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info("behavior_node ready (forward, turn, scan, explore)")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_range(self, name, msg):
        r = msg.range
        d = None if (math.isinf(r) or math.isnan(r) or r >= msg.max_range) else r
        self.dist[name] = d
        self.health.update(name, d, self._now())

    def _emit(self, events):
        for ev in events:
            self.event_pub.publish(String(data=json.dumps(ev)))
            self.get_logger().info("behavior %s: %s" % (ev.get("behavior"), ev.get("result")))

    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError("command must be a JSON object")
        except ValueError as e:
            self._emit([{"id": "", "behavior": None, "result": "error", "data": {"error": str(e)}}])
            return
        self._emit(self.nav.start(cmd))

    def _tick(self):
        now = self._now()
        dt = min(max(now - self.last_t, 0.0), 0.2)
        self.last_t = now
        ok, reason = self.health.status(now)
        r = Ranges.from_optional(self.dist["left"], self.dist["center"], self.dist["right"], ok)

        lin, ang, events = self.nav.tick(dt, r)
        self._emit(events)
        if lin is not None:
            t = Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.cmd_pub.publish(t)

        if now - self.last_state_t >= 0.5:
            self.last_state_t = now
            self.state_pub.publish(String(data=json.dumps({
                "active": self.nav.name if self.nav.active is not None else None,
                "sensors_ok": ok,
                "sensor_reason": reason,
            })))


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
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

"""agent_bridge: exposes the robot to AI agents as a small HTTP API (see api.py).

Hermes uses it through tools/robot_mcp_server.py. Other agents (the claw agent,
the Seeed Watcher) can call the same HTTP endpoints. By default it listens on
127.0.0.1 only. To let another machine call it, set bind_host: "0.0.0.0" AND a
token (parameter `token`, or the environment variable ROBOT_API_TOKEN).
"""

import json
import math
import os
import threading
import time
import uuid
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String

from robot_base.api import ApiServer, EventLog
from robot_base.sensor_health import NAMES


class AgentBridge(Node):
    def __init__(self):
        super().__init__("agent_bridge")
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("token", "")

        self._lock = threading.Lock()
        self._dist = {n: None for n in NAMES}
        self._seen = {n: None for n in NAMES}
        self._safety = "unknown"
        self._behavior_state = {"active": None, "sensors_ok": False, "sensor_reason": "no data yet"}
        self.log = EventLog()

        for n in NAMES:
            self.create_subscription(
                Range, "range/" + n, partial(self._on_range, n), qos_profile_sensor_data
            )
        self.create_subscription(String, "safety/status", self._on_safety, 10)
        self.create_subscription(String, "behavior/state", self._on_state, 10)
        self.create_subscription(String, "behavior/event", self._on_event, 10)
        self.cmd_pub = self.create_publisher(String, "behavior/command", 10)
        self.raw_pub = self.create_publisher(Twist, "cmd_vel_raw", 10)

        host = self.get_parameter("bind_host").value
        port = int(self.get_parameter("port").value)
        token = self.get_parameter("token").value or os.environ.get("ROBOT_API_TOKEN", "")
        self.server = ApiServer(self, host, port, token)
        self.server.start()
        self.get_logger().info(
            "Body API on http://%s:%d (token %s)" % (host, self.server.port, "set" if token else "none")
        )

    # ---- ROS callbacks ----
    def _on_range(self, name, msg):
        r = msg.range
        d = None if (math.isinf(r) or math.isnan(r) or r >= msg.max_range) else round(r, 3)
        with self._lock:
            self._dist[name] = d
            self._seen[name] = time.monotonic()

    def _on_safety(self, msg):
        with self._lock:
            self._safety = msg.data

    def _on_state(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._behavior_state = st

    def _on_event(self, msg):
        try:
            self.log.append(json.loads(msg.data))
        except ValueError:
            pass

    # ---- backend interface used by ApiServer (runs on HTTP threads) ----
    def get_state(self):
        now = time.monotonic()
        with self._lock:
            ages = {n: (None if t is None else round(now - t, 2)) for n, t in self._seen.items()}
            return {
                "ranges_m": dict(self._dist),
                "range_age_s": ages,
                "safety": self._safety,
                "behavior": dict(self._behavior_state),
                "note": "camera cannot see the floor near the car; ultrasonics look forward only",
            }

    def submit(self, cmd):
        cid = uuid.uuid4().hex[:8]
        self.cmd_pub.publish(String(data=json.dumps(dict(cmd, id=cid))))
        return cid

    def drive(self, linear, angular):
        """Direct velocity from a live control loop. The first call cancels any running
        behaviour so the two do not fight over cmd_vel_raw; the safety reflex still applies."""
        with self._lock:
            active = self._behavior_state.get("active")
        if active:
            self.cmd_pub.publish(String(data=json.dumps({"id": "drive", "behavior": "stop"})))
        t = Twist()
        t.linear.x, t.angular.z = float(linear), float(angular)
        self.raw_pub.publish(t)

    def emergency_stop(self):
        self.cmd_pub.publish(String(data=json.dumps({"id": "estop", "behavior": "stop"})))
        self.raw_pub.publish(Twist())  # zero velocity right now


def main(args=None):
    rclpy.init(args=args)
    node = AgentBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

"""motor_bridge: /cmd_vel (Twist) -> UDP "<key> VEL <l> <r>" to the motor ESP32.

The key is a shared secret (file `udp_key_file`, or env ROBOT_UDP_KEY) so other
devices on the same WiFi cannot drive the robot. Without it the ESP32 ignores us.

If no /cmd_vel arrives for `cmd_timeout` seconds it sends STOP instead.
The ESP32 also has its own 500 ms auto-stop, so a dead Pi stops the robot too.
"""

import os
import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from robot_base.mixer import twist_to_wheels
from robot_base.protocol import STOP, format_vel, with_key


class MotorBridge(Node):
    def __init__(self):
        super().__init__("motor_bridge")
        self.declare_parameter("esp32_host", "robot-motors.local")  # name or IP
        self.declare_parameter("esp32_port", 4210)
        self.declare_parameter("max_linear", 0.4)    # m/s that maps to max_pwm
        self.declare_parameter("max_angular", 2.0)   # rad/s that maps to max_pwm
        self.declare_parameter("max_pwm", 200)
        self.declare_parameter("min_pwm", 60)
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("udp_key_file", "")    # file holding the ESP32's ROBOT_KEY

        p = self.get_parameter
        self.host = p("esp32_host").value
        self.port = int(p("esp32_port").value)
        self.max_linear = float(p("max_linear").value)
        self.max_angular = float(p("max_angular").value)
        self.max_pwm = int(p("max_pwm").value)
        self.min_pwm = int(p("min_pwm").value)
        self.cmd_timeout = float(p("cmd_timeout").value)
        rate = float(p("send_rate_hz").value)
        self.key = self._load_key(p("udp_key_file").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.addr = None
        self.last_resolve = -1e9
        self.cmd = Twist()
        self.cmd_time = None
        self.tick = 0
        self.idle_every = max(1, int(rate / 2))  # STOP keep-alive ~2 Hz

        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("motor_bridge -> %s:%d" % (self.host, self.port))

    def _load_key(self, path):
        key = os.environ.get("ROBOT_UDP_KEY", "")
        if not key and path:
            try:
                with open(os.path.expanduser(path)) as f:
                    key = f.read().strip()
            except OSError as e:
                self.get_logger().warn("cannot read udp_key_file %s: %s" % (path, e))
        if not key:
            self.get_logger().warn("no UDP key: the ESP32 will ignore motor commands")
        return key

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = self._now()

    def _resolve(self):
        now = self._now()
        if now - self.last_resolve < 5.0:
            return
        self.last_resolve = now
        try:
            self.addr = (socket.gethostbyname(self.host), self.port)
            self.get_logger().info("resolved %s -> %s" % (self.host, self.addr[0]))
        except OSError:
            self.addr = None
            self.get_logger().warn("cannot resolve %s (is the ESP32 on WiFi?)" % self.host)

    def _send(self, payload):
        if self.addr is None:
            return
        try:
            self.sock.sendto(payload, self.addr)
        except OSError:
            self.addr = None  # force a re-resolve

    def _tick(self):
        if self.addr is None:
            self._resolve()
            return
        self.tick += 1
        fresh = (
            self.cmd_time is not None
            and (self._now() - self.cmd_time) < self.cmd_timeout
        )
        if fresh:
            left, right = twist_to_wheels(
                self.cmd.linear.x, self.cmd.angular.z,
                self.max_linear, self.max_angular, self.max_pwm, self.min_pwm,
            )
            self._send(format_vel(left, right, self.key))
        elif self.tick % self.idle_every == 0:
            self._send(with_key(STOP, self.key))

    def stop_now(self):
        self._send(with_key(STOP, self.key))


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_now()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

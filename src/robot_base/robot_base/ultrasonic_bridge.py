"""ultrasonic_bridge: reads Arduino Uno serial lines "D,l_mm,c_mm,r_mm" and
publishes sensor_msgs/Range on range/left, range/center, range/right.

No echo (-1 from the Uno) is published as +inf, per ROS convention.
Reconnects automatically if the Uno is unplugged or resets.
"""

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from robot_base.protocol import LineBuffer, parse_range_line

NAMES = ("left", "center", "right")


class UltrasonicBridge(Node):
    def __init__(self):
        super().__init__("ultrasonic_bridge")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("min_range", 0.02)
        self.declare_parameter("max_range", 4.0)
        self.declare_parameter("fov", 0.26)  # rad, HC-SR04 is roughly 15 deg
        self.declare_parameter("poll_hz", 50.0)
        # Comma-separated sensors to IGNORE (e.g. "center" when it is broken or blocked): they always
        # report max_range ("clear"), so they neither stop the car nor trip the sensor-health check.
        # The remaining sensors still do. Reduces safety: only for a known-bad sensor.
        self.declare_parameter("ignore_sensors", "")

        p = self.get_parameter
        self.port = p("port").value
        self.baud = int(p("baud").value)
        self.min_range = float(p("min_range").value)
        self.max_range = float(p("max_range").value)
        self.fov = float(p("fov").value)
        self.ignored = {n.strip() for n in str(p("ignore_sensors").value).split(",") if n.strip() in NAMES}
        if self.ignored:
            self.get_logger().warn("IGNORING ultrasonic sensor(s): %s (always reported clear)" % ", ".join(sorted(self.ignored)))

        self.pubs = {
            n: self.create_publisher(Range, "range/" + n, qos_profile_sensor_data)
            for n in NAMES
        }
        self.ser = None
        self.buf = LineBuffer()
        self.last_try = -1e9
        self.create_timer(1.0 / float(p("poll_hz").value), self._poll)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _open(self):
        if self._now() - self.last_try < 2.0:
            return
        self.last_try = self._now()
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.buf = LineBuffer()
            self.get_logger().info("opened %s (Uno resets on open; data in ~2 s)" % self.port)
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn("cannot open %s: %s" % (self.port, e))

    def _poll(self):
        if self.ser is None:
            self._open()
            return
        try:
            data = self.ser.read(256)
        except (serial.SerialException, OSError):
            self.get_logger().warn("serial lost, will reconnect")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return
        for line in self.buf.feed(data):
            vals = parse_range_line(line)
            if vals is not None:
                self._publish(vals)

    def _publish(self, vals):
        stamp = self.get_clock().now().to_msg()
        for name, d in zip(NAMES, vals):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = "ultrasonic_" + name
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.fov
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            if name in self.ignored:
                msg.range = float(self.max_range) * 0.99   # ignored: "clear" (just under max = not "no echo")
            elif d is None or d > self.max_range:
                msg.range = float("inf")
            else:
                msg.range = float(d)
            self.pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicBridge()
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

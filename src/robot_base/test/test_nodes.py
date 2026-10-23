"""Integration test of the ROS nodes against a tiny fake ROS (no ROS needed).

A fake `rclpy` supplies Node/pub/sub/timers/clock on an in-memory bus so the
real node classes run end to end:
    behavior_node -> safety_reflex -> motor_bridge -> UDP packets
plus agent_bridge's HTTP state. Skipped automatically if real ROS is installed.
"""

import importlib
import json
import socket
import sys
import types
import urllib.request

import pytest

try:
    import rclpy  # noqa: F401
    pytest.skip("real ROS present; this test uses a fake one", allow_module_level=True)
except ImportError:
    pass

# ------------------------------------------------------------------ fake ROS
NOW = [0.0]
BUS = {}       # topic -> [callbacks]
HISTORY = {}   # topic -> [messages]
PARAMS = {}    # (node, param) -> override


class _Time:
    def __init__(self):
        self.nanoseconds = int(NOW[0] * 1e9)

    def to_msg(self):
        return 0


class _Clock:
    def now(self):
        return _Time()


class _Log:
    def info(self, *a): pass
    def warn(self, *a): pass
    def error(self, *a): pass


class _Param:
    def __init__(self, v):
        self.value = v


class _Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        HISTORY.setdefault(self.topic, []).append(msg)
        for cb in list(BUS.get(self.topic, [])):
            cb(msg)


class FakeNode:
    ALL = []

    def __init__(self, name):
        self.name, self.params, self.timers = name, {}, []
        FakeNode.ALL.append(self)

    def declare_parameter(self, n, default):
        self.params[n] = PARAMS.get((self.name, n), default)

    def get_parameter(self, n):
        return _Param(self.params[n])

    def create_subscription(self, typ, topic, cb, qos):
        BUS.setdefault(topic, []).append(cb)

    def create_publisher(self, typ, topic, qos):
        return _Pub(topic)

    def create_timer(self, period, cb):
        self.timers.append(cb)

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Log()

    def destroy_node(self):
        pass


class _Vec:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Twist:
    def __init__(self):
        self.linear, self.angular = _Vec(), _Vec()


class _Header:
    stamp, frame_id = 0, ""


class Range:
    ULTRASOUND = 0

    def __init__(self):
        self.header = _Header()
        self.radiation_type = 0
        self.field_of_view = self.min_range = self.max_range = self.range = 0.0


class String:
    def __init__(self, data=""):
        self.data = data


def _mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


_mod("rclpy", init=lambda *a, **k: None, spin=lambda *a, **k: None, ok=lambda: False,
     shutdown=lambda: None)
_mod("rclpy.node", Node=FakeNode)
_mod("rclpy.qos", qos_profile_sensor_data=object())
_mod("geometry_msgs")
_mod("geometry_msgs.msg", Twist=Twist)
_mod("sensor_msgs")
_mod("sensor_msgs.msg", Range=Range)
_mod("std_msgs")
_mod("std_msgs.msg", String=String)
_mod("serial", Serial=object, SerialException=Exception)

# --------------------------------------------------------------------- tests
FAR = float("inf")


@pytest.fixture
def robot():
    NOW[0] = 0.0
    BUS.clear(); HISTORY.clear(); PARAMS.clear(); FakeNode.ALL.clear()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.setblocking(False)
    PARAMS[("motor_bridge", "esp32_host")] = "127.0.0.1"
    PARAMS[("motor_bridge", "esp32_port")] = udp.getsockname()[1]
    PARAMS[("agent_bridge", "port")] = 0

    mods = {}
    for n in ("safety_reflex", "motor_bridge", "behavior_node", "agent_bridge"):
        mods[n] = importlib.import_module("robot_base." + n)
        importlib.reload(mods[n])
    nodes = {
        "safety": mods["safety_reflex"].SafetyReflex(),
        "motor": mods["motor_bridge"].MotorBridge(),
        "behavior": mods["behavior_node"].BehaviorNode(),
        "bridge": mods["agent_bridge"].AgentBridge(),
    }

    class R:
        pass

    r = R()
    r.nodes, r.udp = nodes, udp
    r.ranges = {"left": 2.0, "center": 2.0, "right": 2.0}   # metres, FAR/None = no echo
    r.packets = []

    def publish_ranges():
        for name, d in r.ranges.items():
            m = Range()
            m.max_range = 4.0
            m.range = FAR if d is None else d
            _Pub("range/" + name).publish(m)

    def step(seconds, sensors=True):
        for _ in range(int(round(seconds / 0.05))):
            NOW[0] += 0.05
            if sensors and int(round(NOW[0] / 0.05)) % 2 == 0:   # Uno ~10 Hz
                publish_ranges()
            for n in nodes.values():
                for t in n.timers:
                    t()
            try:
                while True:
                    r.packets.append(udp.recv(100).decode())
            except BlockingIOError:
                pass

    r.step, r.publish_ranges = step, publish_ranges
    yield r
    nodes["bridge"].server.shutdown()
    udp.close()


def command(robot, **cmd):
    nodes = robot.nodes
    return nodes["bridge"].submit(cmd)


def moving(packets):
    """VEL packets that actually move a wheel (idle sends 'VEL 0 0' keepalives)."""
    return [p for p in packets if p.startswith("VEL") and p != "VEL 0 0"]


def events():
    return [json.loads(m.data) for m in HISTORY.get("behavior/event", [])]


def test_forward_goes_through_safety_to_udp_and_finishes(robot):
    robot.step(1.0)
    cid = command(robot, behavior="forward", distance=0.5)
    robot.step(4.0)
    vels = moving(robot.packets)
    assert vels, robot.packets
    l, r = map(int, vels[0].split()[1:])
    assert l > 0 and r > 0 and abs(l - r) <= 2        # straight ahead
    ev = [e for e in events() if e["id"] == cid]
    assert ev and ev[0]["result"] == "ok"
    robot.step(1.5)
    assert robot.packets[-1] in ("VEL 0 0", "STOP")   # stopped after finishing


def test_obstacle_ahead_stops_the_car(robot):
    robot.step(1.0)
    command(robot, behavior="forward", distance=2.0)
    robot.step(1.0)
    assert moving(robot.packets)
    robot.ranges["center"] = 0.1                       # wall appears
    robot.step(0.3)                                    # sensor + control latency (~0.1 s)
    robot.packets.clear()
    robot.step(1.5)
    assert not moving(robot.packets)
    assert any(e["result"] == "blocked" for e in events())


def test_turn_left_commands_opposite_wheels(robot):
    robot.step(1.0)
    command(robot, behavior="turn", degrees=45)
    robot.step(1.0)
    vel = moving(robot.packets)[0]
    l, r = map(int, vel.split()[1:])
    assert l < 0 < r                                    # left wheel back, right forward


def test_unwired_sensors_block_driving(robot):
    # every sensor reports "no echo" forever, like an unpowered HC-SR04
    robot.ranges.update(left=None, center=None, right=None)
    robot.step(10.0)
    robot.packets.clear()
    command(robot, behavior="forward", distance=0.5)
    robot.step(2.0)
    assert not moving(robot.packets)
    assert any(e["result"] == "sensors_unavailable" for e in events())
    status = [m.data for m in HISTORY.get("safety/status", [])]
    state = json.loads(HISTORY["behavior/state"][-1].data)
    assert state["sensors_ok"] is False and "stuck" in state["sensor_reason"]


def test_no_sensor_data_blocks_driving(robot):
    robot.step(1.0, sensors=False)                      # Uno unplugged
    command(robot, behavior="forward", distance=0.5)
    robot.step(2.0, sensors=False)
    assert not moving(robot.packets)


def test_teleop_path_is_also_protected(robot):
    robot.step(1.0)
    robot.ranges["center"] = 0.15
    robot.step(0.5)
    t = Twist(); t.linear.x = 0.3
    for _ in range(20):                                 # operator holds the key
        _Pub("cmd_vel_raw").publish(t)
        robot.step(0.05)
    assert not moving(robot.packets)


def test_state_endpoint_reflects_ranges(robot):
    robot.ranges.update(left=1.5, center=None, right=0.6)
    robot.step(1.0)
    st = robot.nodes["bridge"].get_state()
    assert st["ranges_m"] == {"left": 1.5, "center": None, "right": 0.6}
    assert st["behavior"]["sensors_ok"] is True
    port = robot.nodes["bridge"].server.port
    with urllib.request.urlopen("http://127.0.0.1:%d/health" % port, timeout=3) as r:
        assert json.loads(r.read()) == {"ok": True}


def test_emergency_stop_preempts_explore(robot):
    robot.step(1.0)
    command(robot, behavior="explore", seconds=60)
    robot.step(1.0)
    assert moving(robot.packets)
    robot.nodes["bridge"].emergency_stop()
    robot.packets.clear()
    robot.step(1.5)
    assert not moving(robot.packets)
    assert any(e["result"] == "preempted" for e in events())

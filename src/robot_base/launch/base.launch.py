import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    cfg = os.path.join(get_package_share_directory("robot_base"), "config", "base.yaml")
    motors = LaunchConfiguration("motors").perform(context).lower() in ("true", "1", "yes")

    def node(executable, extra=None):
        params = [cfg] + ([extra] if extra else [])
        return Node(package="robot_base", executable=executable, name=executable,
                    parameters=params, output="screen")

    # motors:=false (default) sends motor packets to 127.0.0.1 where nothing listens,
    # so the ESP32 never receives anything. Use motors:=true only with the battery
    # connected and the wheels off the ground (first test) or on the floor.
    motor_extra = None if motors else {"esp32_host": "127.0.0.1"}
    return [
        LogInfo(msg="MOTORS %s" % ("ENABLED" if motors else "DISABLED (motors:=false)")),
        node("ultrasonic_bridge"),           # Uno -> range/left|center|right
        node("safety_reflex"),               # cmd_vel_raw -> cmd_vel (obstacle + sensor-fault protection)
        node("motor_bridge", motor_extra),   # cmd_vel -> ESP32
        node("behavior_node"),               # forward / turn / scan / explore
        node("agent_bridge"),                # HTTP Body API for Hermes and other agents
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("motors", default_value="false",
                              description="true = send motor commands to the ESP32"),
        OpaqueFunction(function=_nodes),
    ])

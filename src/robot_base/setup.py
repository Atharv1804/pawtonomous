from glob import glob

from setuptools import find_packages, setup

package_name = "robot_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Atharv",
    maintainer_email="atharv.chaurasia1804@gmail.com",
    description="Body layer for the companion robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_bridge = robot_base.motor_bridge:main",
            "ultrasonic_bridge = robot_base.ultrasonic_bridge:main",
            "safety_reflex = robot_base.safety_reflex:main",
            "behavior_node = robot_base.behavior_node:main",
            "agent_bridge = robot_base.agent_bridge:main",
        ],
    },
)

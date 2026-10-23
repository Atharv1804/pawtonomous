#!/bin/bash
# Run the robot software (ROS 2 in Docker) on the Pi.
#   ~/robot_ws/docker/robot.sh build    build the Docker image + the ROS package
#   ~/robot_ws/docker/robot.sh start    start (motors OFF); also restarts by itself after a reboot
#   ~/robot_ws/docker/robot.sh stop     stop
#   ~/robot_ws/docker/robot.sh logs     show what it is doing (Ctrl+C to quit)
#   ~/robot_ws/docker/robot.sh shell    open a ROS shell inside it (ros2 topic echo ...)
# Motors stay disabled unless MOTORS=true is set:  MOTORS=true ~/robot_ws/docker/robot.sh start
set -e
WS="$HOME/robot_ws"
IMAGE=robot_ws:jazzy
NAME=robot
MOTORS="${MOTORS:-false}"

case "$1" in
  build)
    docker build -t "$IMAGE" "$WS/docker"
    docker run --rm -v "$WS:/ws" "$IMAGE" bash -c \
      "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build && chown -R $(id -u):$(id -g) build install log"
    ;;
  start)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    # /dev is shared (not single --device) so the Uno can be unplugged/moved to any USB port
    # and found again by its /dev/serial/by-id name. 166 = ttyACM*, 188 = ttyUSB*.
    docker run -d --name "$NAME" --restart unless-stopped --network host \
      -v /dev:/dev --device-cgroup-rule='c 166:* rmw' --device-cgroup-rule='c 188:* rmw' \
      -v "$WS:/ws" "$IMAGE" bash -c \
      "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec ros2 launch robot_base base.launch.py motors:=$MOTORS"
    echo "started (motors: $MOTORS)"
    ;;
  stop)  docker rm -f "$NAME" >/dev/null && echo stopped ;;
  logs)  docker logs -f --tail 50 "$NAME" ;;
  shell) docker exec -it "$NAME" bash ;;
  *) sed -n '2,8p' "$0" ;;
esac

# ROS 2 Jazzy + the extras robot_base needs. Build on the Pi:
#   docker build -t robot_ws:jazzy ~/robot_ws/docker
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial python3-pytest ros-jazzy-teleop-twist-keyboard curl \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc
WORKDIR /ws
# avrdude: flash the Uno from the Pi (no Mac/cable needed), see docker/flash_uno.sh
RUN apt-get update && apt-get install -y --no-install-recommends avrdude \
    && rm -rf /var/lib/apt/lists/*

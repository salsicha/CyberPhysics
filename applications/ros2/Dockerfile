
# ROS2 Jazzy

FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# ROS2
RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*
RUN curl --insecure -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
RUN apt update && apt install -q -y --no-install-recommends \
    python3-serial imagemagick libboost-all-dev libgts-dev libjansson-dev ros-$ROS2_DISTRO-rviz2 \
    ros-$ROS2_DISTRO-foxglove-bridge ros-$ROS2_DISTRO-foxglove-compressed-video-transport ros-$ROS2_DISTRO-foxglove-msgs \
    ros-$ROS2_DISTRO-ros-core ros-$ROS2_DISTRO-sensor-msgs-py libeigen3-dev \
    ros-$ROS2_DISTRO-ros2bag ros-$ROS2_DISTRO-rclpy ros-$ROS2_DISTRO-rosbag2-storage-default-plugins \
    python3-rosdep python3-colcon-ros linuxptp python3-colcon-common-extensions \
    ros-$ROS2_DISTRO-rosbridge-suite ros-$ROS2_DISTRO-rosbag2 ros-$ROS2_DISTRO-gps-msgs \
    ros-$ROS2_DISTRO-ros2-control vim git jq \
    ros-$ROS2_DISTRO-tf2-msgs software-properties-common build-essential gcc && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv
# EM is preventing msgs from building...
RUN pip3 uninstall em
RUN pip3 install rosdep colcon-common-extensions pytest-rerunfailures numpy lark empy==3.3.4

WORKDIR /workspace
RUN mkdir -p /workspace/src
COPY custom_msgs/ /workspace/src/

RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --merge-install --install-base /opt/ros/$ROS2_DISTRO

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

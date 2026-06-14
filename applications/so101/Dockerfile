FROM ros:jazzy-ros-base AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-numpy \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-controller-manager-msgs \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-position-controllers \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-transmission-interface \
    ros-$ROS_DISTRO-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/src/so101_description
COPY . .

WORKDIR /workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    xacro src/so101_description/urdf/so101.urdf.xacro use_gazebo:=false -o src/so101_description/urdf/so101.urdf && \
    colcon build --packages-select so101_description --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "so101_description", "gazebo.launch.py"]

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
    python3-pip \
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

RUN pip3 install --break-system-packages --no-cache-dir msgpack msgpack-numpy pyzmq

WORKDIR /workspace/src/so101_description
COPY . .

WORKDIR /workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select so101_description --symlink-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "so101_description", "gazebo.launch.py"]

# Policy evaluation image (cyberphysics/lerobot), selected with
# --target lerobot (BUILD_STAGE=lerobot via _build_it.sh). Not part of
# the default build, so cyberphysics/so101 stays lean.
FROM build AS lerobot

# Install the policy stack in an isolated venv so pip does not collide with
# the Debian/apt-managed system packages (uninstalling e.g. python3-packaging
# fails with "RECORD file not found"). The evaluation scripts are pure
# lerobot/gym and do not need the ROS packages. pymunk<7 keeps gym-pusht's
# collision handler API working.
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/lerobot_venv && \
    /opt/lerobot_venv/bin/pip install --no-cache-dir --upgrade pip && \
    /opt/lerobot_venv/bin/pip install --no-cache-dir \
    "lerobot[smolvla,pi0,diffusion]" \
    gym-pusht \
    "pymunk<7" \
    "imageio[ffmpeg]"

WORKDIR /workspace/src/so101_description/scripts
CMD ["/opt/lerobot_venv/bin/python3", "evaluate_pretrained_policy.py"]

# Default target: the plain so101 simulation image.
FROM build

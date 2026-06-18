FROM cyberphysics/ros2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ISAAC_ROS_RELEASE_MAJOR=4 \
    ISAAC_ROS_RELEASE_MINOR=4 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video \
    PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH} \
    DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    ca-certificates curl gnupg jq tar \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
      -o /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && \
    rm /tmp/cuda-keyring.deb && \
    curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-jetson.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.4 main" \
      > /etc/apt/sources.list.d/nvidia-jetson.list && \
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
      | gpg --dearmor -o /usr/share/keyrings/isaac-ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4.4 noble main" \
      > /etc/apt/sources.list.d/isaac-ros.list

RUN apt update && apt install -q -y --no-install-recommends \
    ros-${ROS2_DISTRO}-isaac-ros-nvblox \
    ros-${ROS2_DISTRO}-nav2-bringup \
    ros-${ROS2_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

COPY setup.sh /setup.sh
RUN chmod +x /setup.sh && /setup.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ISAAC_ROS_RELEASE_MAJOR=4 \
    ISAAC_ROS_RELEASE_MINOR=4 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video \
    PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH}

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "nvblox_examples_bringup", "isaac_sim_example.launch.py", "rosbag:=/workspace/isaac_ros_assets/isaac_ros_nvblox/quickstart", "navigation:=False", "run_rviz:=False"]

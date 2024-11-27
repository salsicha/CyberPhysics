
FROM cyberphysics/ros2:latest AS build

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

# Install required dependencies for the intermediate container
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl unzip ca-certificates jq tar sudo wget && \
    rm -rf /var/lib/apt/lists/*

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb
RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    cuda-toolkit nvidia-gds && \
    rm -rf /var/lib/apt/lists/*

# Quickstart assets:
COPY setup.sh /setup.sh
RUN chmod +x /setup.sh
RUN /setup.sh

# Build and install nvblox
WORKDIR /workspace/src/

RUN curl -L -o isaac_ros_common.zip https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/archive/refs/tags/v3.1.0.zip && \
    unzip isaac_ros_common.zip && rm isaac_ros_common.zip && mv isaac_ros_common-* isaac_ros_common
RUN curl -L -o nvblox.zip https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/archive/refs/tags/v3.1.0.zip && \
    unzip nvblox.zip && rm nvblox.zip && mv isaac_ros_nvblox-* isaac_ros_nvblox

# Install VPI packages
RUN wget https://repo.download.nvidia.com/jetson/x86_64/jammy/pool/main/libn/libnvvpi3/libnvvpi3_3.2.4_amd64.deb
RUN dpkg -i libnvvpi*

WORKDIR /workspace/

RUN apt-get update && \
    . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    rosdep update && \
    rosdep install -i -r --from-paths /workspace/src/isaac_ros_nvblox/ --rosdistro $ROS2_DISTRO -y && \
    colcon build --symlink-install --packages-up-to isaac_ros_nvblox

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
CMD ["ros2", "launch nvblox_examples_bringup", "isaac_sim_example.launch.py", "rosbag:=/workspace/isaac_ros_assets/isaac_ros_nvblox/quickstart", "navigation:=False"]

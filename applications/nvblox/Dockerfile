
FROM cyberphysics/ros2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# Install required dependencies for the intermediate container
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl unzip ca-certificates jq tar && \
        rm -rf /var/lib/apt/lists/*

# Quickstart assets:
COPY setup.sh /setup.sh
RUN chmod +x /setup.sh
RUN /setup.sh

# Build and install nvblox
WORKDIR /workspace/src/
RUN curl -L -o nvblox.zip https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/archive/refs/tags/v3.1.0.zip && \
    unzip nvblox.zip && rm nvblox.zip && mv isaac_ros_nvblox-* isaac_ros_nvblox
WORKDIR /workspace/
RUN ./src/isaac_ros_common/scripts/run_dev.sh
RUN apt-get update
RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    rosdep update && \
    rosdep install -i -r --from-paths /workspaces/src/isaac_ros_nvblox/ --rosdistro $ROS2_DISTRO -y && \
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


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

RUN apt update && apt install -q -y --no-install-recommends \
    apt install ros-jazzy-ros-gz && \
    rm -rf /var/lib/apt/lists/*

# RUN mkdir -p /workspace/src
# COPY ros2_ws/ /workspace/src/
# RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
#     rosdep init && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -y && \
#     colcon build --merge-install --install-base /opt/ros/$ROS2_DISTRO

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

# ENTRYPOINT ["/entrypoint.sh"]

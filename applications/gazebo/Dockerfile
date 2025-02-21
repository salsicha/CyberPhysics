
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
    lsb-release gnupg && \
    rm -rf /var/lib/apt/lists/*

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update && apt install -q -y --no-install-recommends \
    gz-ionic ros-${ROS2_DISTRO}-joint-state-publisher ros-${ROS2_DISTRO}-joy \
    ros-${ROS2_DISTRO}-robot-state-publisher ros-${ROS2_DISTRO}-teleop-twist-keyboard \
    ros-${ROS2_DISTRO}-imu-tools ros-${ROS2_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspace/src
COPY ros2_ws/ /workspace/src/
WORKDIR /workspace
RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    rosdep update
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

ENTRYPOINT ["/entrypoint.sh"]

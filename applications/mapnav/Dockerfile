ARG TAG=amd64

FROM cyberphysics/ros2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=xcb \
    ROS_DISTRO=jazzy

WORKDIR /workspace

# Micro-ROS agent
RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    apt update && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build --merge-install --install-base /opt/ros/$ROS2_DISTRO

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]



FROM cyberphysics/ros2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=xcb \
    ROS_DISTRO=jazzy

COPY requirements.txt /

RUN apt update && apt install -y --no-install-recommends \
    python3-rosdep python3-colcon-ros python3-pip ros-$ROS2_DISTRO-cv-bridge \
    && apt install -y x11-apps qtwayland5 \
    && . /venv/bin/activate \
    && pip install -r /requirements.txt \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get upgrade -y

COPY scripts/ /scripts/
WORKDIR /scripts

RUN chmod +x /scripts/install_deps.sh
RUN apt update && /scripts/install_deps.sh

RUN . /venv/bin/activate && pip install depthai-viewer

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

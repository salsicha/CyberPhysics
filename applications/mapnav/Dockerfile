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

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    ros-$ROS_DISTRO-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN . /venv/bin/activate && pip install --no-cache-dir \
    numpy \
    opencv-python \
    "srtm.py"

COPY src /workspace/src

RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    apt-get update && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build --merge-install --install-base /opt/ros/$ROS2_DISTRO

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

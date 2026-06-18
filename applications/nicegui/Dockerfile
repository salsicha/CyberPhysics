ARG TAG=amd64

FROM cyberphysics/ros2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-opencv \
    build-essential \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-nav2-msgs \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
WORKDIR /workspace

RUN python3 -m venv --system-site-packages /venv && \
    . /venv/bin/activate && \
    pip3 install --no-cache-dir "numpy<2" nicegui

COPY scripts /workspace/scripts
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

EXPOSE 8080
ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/workspace/scripts/main.py"]

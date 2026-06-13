ARG TAG=amd64

FROM cyberphysics/aerostack2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace
COPY src /workspace/src

RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    colcon build --merge-install --install-base /workspace/install

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

FROM scratch
COPY --from=build / /

ENTRYPOINT ["/entrypoint.sh"]

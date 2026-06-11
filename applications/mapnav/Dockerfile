ARG TAG=amd64

FROM cyberphysics/aerostack2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=xcb \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace

RUN --mount=type=cache,target=/root/.cache/pip \
    . /venv/bin/activate && \
    pip install --timeout 300 --retries 10 "srtm.py"

COPY src /workspace/src

RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    colcon build --merge-install --install-base /workspace/install

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

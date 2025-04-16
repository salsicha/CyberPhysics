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

# RUN apt update && apt install -y --no-install-recommends \
#     ? \
#     && rm -rf /var/lib/apt/lists/*

COPY scripts/ /scripts/

WORKDIR /scripts

RUN . /venv/bin/activate && pip install smbus smbus2 spidev

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

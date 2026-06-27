ARG TAG=amd64

FROM cyberphysics/aerostack2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

COPY scripts/ /scripts/
RUN chmod +x /scripts/*.py

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

FROM scratch

COPY --from=build / /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/scripts/bluerov2_bridge.py"]

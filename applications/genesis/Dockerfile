FROM cyberphysics/ros2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

SHELL ["/bin/bash", "-c"]

COPY scripts/ /scripts/

RUN python3 -m venv /venv
RUN . /venv/bin/activate && pip3 install torch  --index-url https://download.pytorch.org/whl/cu121
RUN . /venv/bin/activate && pip3 install genesis-world

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

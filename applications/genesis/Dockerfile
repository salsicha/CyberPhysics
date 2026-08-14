FROM cyberphysics/aerostack2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

SHELL ["/bin/bash", "-c"]

COPY scripts/ /scripts/

RUN python3 -m venv /venv
RUN . /venv/bin/activate && pip3 install 'torch>=2.8,<2.9' --index-url https://download.pytorch.org/whl/cu128
RUN . /venv/bin/activate && pip3 install genesis-world==1.3.3

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

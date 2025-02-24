
FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN python3 -m venv /venv
RUN . /venv/bin/activate && pip3 install torch  --index-url https://download.pytorch.org/whl/cu121
RUN . /venv/bin/activate && pip3 install genesis-world

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]


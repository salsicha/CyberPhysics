
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
RUN . /venv/bin/activate && pip3 install torch --index-url https://download.pytorch.org/whl/cu121
RUN . /venv/bin/activate && pip3 install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com

RUN . /venv/bin/activate && cd /PegasusSimulator/extensions/ && ISAACSIM_PYTHON -m pip install --editable pegasus.simulator

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

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "gui", "main_launch.py"]



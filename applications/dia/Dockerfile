
# ROS2 Jazzy

FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim git software-properties-common build-essential gcc && \
    rm -rf /var/lib/apt/lists/*

COPY scripts/ /scripts/

RUN python3 -m venv /venv
RUN pip3 install setuptools --upgrade

# RUN . /venv/bin/activate && pip3 install git+https://github.com/nari-labs/dia.git

RUN git clone https://github.com/nari-labs/dia.git /dia

WORKDIR /dia

RUN . /venv/bin/activate && pip3 install -e .

COPY entrypoint.sh /entrypoint.sh
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

WORKDIR /dia

ENTRYPOINT ["/entrypoint.sh"]

CMD [ "python3", "app.py" ]

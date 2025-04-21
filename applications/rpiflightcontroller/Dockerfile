
## Build w/:
## make target_arm64 build_oak1

ARG TAG=amd64

# FROM cyberphysics/ros2:${TAG} AS build
FROM ros:humble-ros-base AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=humble

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release vim wget python3-pip python3-venv python3-setuptools \
    ros-${ROS_DISTRO}-aerostack2 ros-${ROS_DISTRO}-as2-platform-gazebo ros-${ROS_DISTRO}-as2-gazebo-assets \
    ros-${ROS_DISTRO}-as2-platform-tello && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv
# RUN . /venv/bin/activate && pip3 install rpi-hardware-pwm smbus smbus2 \
#     board adafruit-extended-bus lgpio adafruit-circuitpython-icm20x adafruit-circuitpython-bme280 \
#     rsa PySimpleGUI-4-foss

COPY requirements.txt /requirements.txt

RUN . /venv/bin/activate && pip3 install -r /requirements.txt

COPY scripts/ /scripts/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

CMD ["python3", "/scripts/scout/python/ros_flight_controller.py", "False"]

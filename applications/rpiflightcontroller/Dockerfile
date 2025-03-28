
FROM cyberphysics/ros2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# RUN apt update && apt install -q -y --no-install-recommends \
#      \
#      && \
#     rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv
RUN . /venv/bin/activate && pip3 install rpi-hardware-pwm smbus smbus2

COPY scripts/ /scripts/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

CMD ["python3", "/scripts/scout/python/ros_flight_controller.py", "False"]

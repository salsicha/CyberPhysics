
FROM ros:humble AS build

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
    curl gnupg2 lsb-release vim wget ros-$ROS_DISTRO-aerostack2 \
     && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install rpi-hardware-pwm smbus smbus2

COPY scripts/ /scripts/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

# Launch aerostack2 simulated drone platform 
CMD ["ros2", "launch", "as2_platform_gazebo", "platform_gazebo_launch.py", "namespace:=drone_sim_0", "simulation_config_file:=world_json_path"]

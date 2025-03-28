
FROM cyberphysics/ros2:latest AS build

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
    curl gnupg2 lsb-release vim wget \
    ros-{ROS_DISTRO}-as2-platform-gazebo ros-{ROS_DISTRO}-ros-gzharmonic && \
    rm -rf /var/lib/apt/lists/*


RUN python3 -m venv /venv


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

# Launch Aerostack2 gazebo simulator
CMD ["ros2", "launch", "as2_gazebo_assets", "launch_simulation.py", "simulation_config_file:=world_json_path"]

ARG TAG=amd64

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
    curl gnupg2 lsb-release vim wget python3-pip \
    ros-${ROS_DISTRO}-aerostack2 ros-${ROS_DISTRO}-as2-platform-gazebo ros-${ROS_DISTRO}-as2-gazebo-assets \
    ros-${ROS_DISTRO}-as2-platform-tello && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install rsa PySimpleGUI-4-foss

COPY project_as2_multirotor_simulator/config/ /workspace/config/
COPY project_as2_multirotor_simulator/config_ground_station/ /workspace/config_ground_station/
COPY project_as2_multirotor_simulator/rosbag/ /workspace/rosbag/
COPY project_as2_multirotor_simulator/trees/ /workspace/trees/
COPY project_as2_multirotor_simulator/utils/ /workspace/utils/
COPY project_as2_multirotor_simulator/tmuxinator/ /workspace/tmuxinator/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

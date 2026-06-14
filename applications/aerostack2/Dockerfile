ARG TAG=amd64

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
    python3-pip python3-venv python3-setuptools libgl1-mesa-dri \
    ros-${ROS_DISTRO}-as2-core ros-${ROS_DISTRO}-as2-msgs \
    ros-${ROS_DISTRO}-as2-state-estimator \
    ros-${ROS_DISTRO}-as2-motion-controller \
    ros-${ROS_DISTRO}-as2-behaviors-motion \
    ros-${ROS_DISTRO}-as2-behaviors-trajectory-generation \
    ros-${ROS_DISTRO}-as2-behaviors-perception \
    ros-${ROS_DISTRO}-as2-behavior-tree \
    ros-${ROS_DISTRO}-as2-platform-gazebo \
    ros-${ROS_DISTRO}-as2-gazebo-assets \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-rosbridge-suite \
    geographiclib-tools python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY requirements.txt /requirements.txt
RUN python3 -m venv --system-site-packages /venv
RUN . /venv/bin/activate && pip3 install -r /requirements.txt

COPY src /workspace/src
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /workspace && \
    colcon build --merge-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

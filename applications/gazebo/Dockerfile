ARG ROS_DISTRO=iron
FROM ros:${ROS_DISTRO}-ros-core-jammy as build

SHELL ["/bin/bash","-c"]

RUN apt-get update \
    && apt-get install -y \
    wget curl unzip \
    lsb-release \
    mesa-utils \
    build-essential \
    && apt-get clean

# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y \
    curl gnupg2 lsb-release python3-pip python3-venv gazebo \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-joy \
    python3-colcon-common-extensions python3-rosdep --no-install-recommends \
    && apt-get clean
RUN rosdep init && rosdep update

RUN python3 -m venv /venv

RUN mkdir -p /workspace/src
COPY ros2_ws/src/ /workspace/src/
WORKDIR /workspace

RUN curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp && mkdir -p ~/.gazebo/models/ && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip

# RUN . /venv/bin/activate && . /opt/ros/${ROS_DISTRO}/setup.bash && \
#     rosdep init && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -y && \
#     colcon build --merge-install --install-base /opt/ros/${ROS_DISTRO} && \
#     apt-get clean

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################
FROM scratch

COPY --from=build / /

WORKDIR /workspace

# ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "sjtu_drone_bringup", "sjtu_drone_bringup.launch.py"]

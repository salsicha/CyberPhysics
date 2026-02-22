FROM ubuntu:focal

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8

RUN usermod -a -G dialout root

RUN ln -snf /bin/bash /bin/sh

RUN apt update && apt install -q -y --no-install-recommends tzdata \
    dirmngr gnupg2 lsb-release ca-certificates rsync \
    vim git curl imagemagick xvfb libjansson-dev locales locales-all \
    build-essential ninja-build keyboard-configuration

# ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install packages
RUN apt update && apt install -q -y --no-install-recommends \
    build-essential cmake ros-noetic-gazebo-ros python3-serial \
    imagemagick libboost-all-dev libgts-dev libjansson-dev \
    libtinyxml-dev nodejs npm pkg-config psmisc ros-noetic-ros-core rviz \
    geographiclib-tools libgeographic-dev ros-noetic-geographic-info ros-noetic-geographic-msgs \
    ros-noetic-mavlink ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-mavros-extras ros-noetic-ros-numpy \
    ros-noetic-velodyne-gazebo-plugins ros-noetic-velodyne-description \
    ros-noetic-py-trees-msgs ros-noetic-py-trees-ros ros-noetic-py-trees ros-noetic-rqt-py-trees \
    ros-noetic-rospy ros-noetic-rospy-message-converter ros-noetic-gazebo-msgs && \
    /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -q -y --no-install-recommends ros-foxy-desktop \
  ros-foxy-py-trees-ros-viewer xdot \
  ros-foxy-ros1-bridge python3-argcomplete python3-pip \
  ros-foxy-py-trees ros-foxy-py-trees-ros ros-foxy-sensor-msgs-py \
  ros-foxy-point-cloud-msg-wrapper && \
  rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -q -y --no-install-recommends ros-foxy-desktop \
  python3-cherrypy3 python3-geopy python3-scipy python3-simplejson python3-jinja2 && \
  rm -rf /var/lib/apt/lists/*

RUN pip install pid pymavlink dronekit dronekit-sitl -UI

COPY scripts/ros_bridge.sh /ros_bridge.sh
COPY scripts/behaviortree.sh /behaviortree.sh

RUN mkdir -p /workspace/misc/lamp_autonomy
COPY . /workspace/misc/lamp_autonomy

WORKDIR /workspace/misc/lamp_autonomy

RUN echo '#!/bin/bash' >> /entrypoint.sh
RUN echo 'set -e' >> /entrypoint.sh
RUN echo '/ros_bridge.sh &' >> /entrypoint.sh
RUN echo 'roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 &' >> /entrypoint.sh
RUN echo 'rosservice call /mavros/set_stream_rate 0 10 1' >> /entrypoint.sh
RUN echo 'exec "$@"' >> /entrypoint.sh
RUN chmod +x /entrypoint.sh

# RUN sed -i '/^exec*/i rosservice call /mavros/set_stream_rate 0 10 1' /entrypoint.sh
# RUN sed -i '/^exec*/i roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 &' /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/behaviortree.sh"]



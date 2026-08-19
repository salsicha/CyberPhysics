ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -q -y --no-install-recommends \
      build-essential \
      cmake \
      libboost-all-dev \
      libceres-dev \
      libeigen3-dev \
      libopencv-dev \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY vins_mono_ros2/ src/vins_mono_ros2/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
      --merge-install \
      --packages-select \
        camera_model \
        config_pkg \
        feature_tracker \
        vins_estimator \
        pose_graph \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /data/vinsmono

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "vins_estimator", "vins_mono.launch.py", \
     "config_file:=/etc/vinsmono/calibration.yaml"]



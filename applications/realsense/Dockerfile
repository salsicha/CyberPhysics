
FROM cyberphysics/ros2:latest AS build

ENV LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy

RUN apt update && apt install -q -y --no-install-recommends \
    ros-$ROS2_DISTRO-realsense2-camera-msgs ros-$ROS2_DISTRO-realsense2-description \
    ros-$ROS2_DISTRO-realsense2-camera ros-$ROS2_DISTRO-realsense2-camera-msgs-dbgsym \
    ros-$ROS2_DISTRO-realsense2-camera-dbgsym \
    ros-$ROS2_DISTRO-librealsense2 ros-$ROS2_DISTRO-librealsense2-dbgsym && \
    rm -rf /var/lib/apt/lists/*
    
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy

COPY --from=build / /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2 launch realsense2_camera rs_launch.py"]

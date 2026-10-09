ARG TAG=latest

FROM salsicha/cyberphysics:ros2-${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash","-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl unzip ca-certificates vim && \
    rm -rf /var/lib/apt/lists/* && \
    ln -s /usr/bin/python3 /usr/bin/python

# salsicha/tello-ros2 (a fork of tentone/tello-ros2) main as of 2024-11-27
ARG TELLO_ROS2_COMMIT=672fa9ec31734d577af358393ec653c287418398

RUN curl -fL -o /tello-ros2.zip \
      https://github.com/salsicha/tello-ros2/archive/${TELLO_ROS2_COMMIT}.zip && \
    unzip -q /tello-ros2.zip -d / && rm /tello-ros2.zip && \
    mv /tello-ros2-${TELLO_ROS2_COMMIT} /tello-ros2

# The fork's scripts/install.sh installs a ROS desktop, rviz/rqt and unpinned
# pip packages; the base image already has ROS (rclpy, cv_bridge, tf2_ros,
# OpenCV for tello_control), so only the driver's Python imports are added.
# djitellopy2 is the DJITelloPy release install.sh names (its frames are BGR,
# as the driver publishes them); --no-deps keeps its GUI opencv-python out.
# numpy 1.x: ROS's cv_bridge extension is built against Ubuntu's numpy 1.26.
RUN . /venv/bin/activate && \
    pip install --no-cache-dir \
      numpy==1.26.4 \
      opencv-python-headless==4.11.0.86 \
      av==12.3.0 && \
    pip install --no-cache-dir --no-deps djitellopy2==2.3

# tello (driver), tello_msg (status messages), tello_control (keyboard teleop)
WORKDIR /tello-ros2/workspace
RUN . /venv/bin/activate && source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select tello_msg tello tello_control

COPY tello.launch.py tello_driver.py /tello-ros2/
RUN chmod +x /tello-ros2/tello_driver.py
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

COPY --from=build / /

WORKDIR /tello-ros2

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/tello-ros2/tello.launch.py"]

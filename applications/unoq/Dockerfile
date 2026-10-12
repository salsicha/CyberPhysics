# Small ROS 2 Jazzy image for the Arduino UNO Q's Linux side (Debian on a
# quad Cortex-A53 with 2 or 4 GB RAM and 16 or 32 GB eMMC): rclpy and the
# common message packages from the official ros-core image, plus msgpack for
# the arduino-router socket (RouterBridge RPC to the board's STM32 MCU) and
# pyserial. The repository's ros2 image (rviz2, Boost, the micro-ROS agent) is
# about ten times larger.
#
# Scripts are mounted from the repository by the compositions:
#   compositions/turret_uno_q.yaml  applications/pantiltros/scripts/turret_arduino_bridge.py
#   compositions/arduino_uno_q.yaml applications/arduino/scripts/unoq_sub/unoq_sub_node.py
#
# The image is multi-arch: build it on the UNO Q itself (compose builds it on
# first `up`) or `make -C applications target_arm64 build_unoq` on a host with
# arm64 emulation.

FROM ros:jazzy-ros-core

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS2_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -q -y --no-install-recommends \
      python3-msgpack \
      python3-serial \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /scripts

ENTRYPOINT ["/entrypoint.sh"]
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_ROUTER_CHECK_ATTEMPTS=-1 \
    ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true;listen/endpoints=["tcp/0.0.0.0:0"];connect/endpoints=[]'
CMD ["bash"]

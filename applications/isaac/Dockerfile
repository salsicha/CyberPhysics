ARG ISAAC_SIM_VERSION=6.0.1
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION}

LABEL org.opencontainers.image.title="CyberPhysics Isaac Sim"
LABEL org.opencontainers.image.description="Isaac Sim 6.0.1 runtime with CyberPhysics integration assets"

USER root

# Isaac's bundled ROS libraries do not contain rmw_zenoh. Use the external
# Ubuntu 24.04 / Python 3.12 Jazzy installation for every simulator bridge.
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main' \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base ros-jazzy-rmw-zenoh-cpp ros-jazzy-vision-msgs && \
    rm -rf /var/lib/apt/lists/*
ENV ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_ROUTER_CHECK_ATTEMPTS=-1 \
    ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true;listen/endpoints=["tcp/0.0.0.0:0"];connect/endpoints=[]'

COPY config/ /workspace/applications/isaac/config/
COPY scripts/ /workspace/applications/isaac/scripts/

RUN chmod +x /workspace/applications/isaac/scripts/*.py /workspace/applications/isaac/scripts/*.sh

USER isaac-sim

WORKDIR /isaac-sim
ENTRYPOINT ["/workspace/applications/isaac/scripts/with_ros.sh"]

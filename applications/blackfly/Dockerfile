# Cross platform support
ARG TAG=latest
FROM --platform=$TARGETPLATFORM salsicha/cyberphysics:ros2-${TAG}

ARG TARGETARCH

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# Install development tools
RUN apt update && apt install -q -y --no-install-recommends \
    software-properties-common build-essential libgomp1 && \
    rm -rf /var/lib/apt/lists/*

# Accept the EULA and install the Spinnaker SDK. The SDK tarballs are
# bind-mounted and unpacked in /tmp, and removed in the same step, so neither
# is kept in a layer.
RUN --mount=type=bind,source=files,target=/tmp/spinnaker-files \
    echo 'libgentl libspinnaker/accepted-flir-eula boolean true' | debconf-set-selections && \
    apt update && \
    mkdir -p /tmp/spinnaker && \
    tar xzvf /tmp/spinnaker-files/spinnaker-3.2.0.57-${TARGETARCH}-pkg-22.04.tar.gz -C /tmp/spinnaker && \
    cd /tmp/spinnaker/spinnaker-3.2.0.57-${TARGETARCH} && \
    apt -y install ./libgentl_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker-dev_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker-c_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker-c-dev_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinvideo_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinvideo-c_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinvideo-dev_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./spinnaker_3.2.0.57_${TARGETARCH}.deb && \
    cd / && rm -rf /tmp/spinnaker /var/lib/apt/lists/*

COPY src /workspace/src
WORKDIR /workspace

# enable cache
RUN --mount=type=cache,target=/var/cache/apt \
    . /opt/ros/$ROS2_DISTRO/setup.bash && \
    apt update && \
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro $ROS2_DISTRO && \
    rm -rf /var/lib/apt/lists/*

# colcon --merge-install into /opt/ros replaces ROS's setup scripts with ones
# that only know the packages built here (dropping the apt packages' library
# paths); restore the originals, which still find these packages through the
# ament index (same as applications/ros2/Dockerfile).
RUN . /opt/ros/$ROS2_DISTRO/setup.bash && \
    mkdir -p /tmp/ros_setup_orig && \
    cp -a /opt/ros/$ROS2_DISTRO/*setup.* /opt/ros/$ROS2_DISTRO/_local_setup_util.py /tmp/ros_setup_orig/ && \
    colcon build --merge-install --install-base /opt/ros/$ROS2_DISTRO && \
    cp -a /tmp/ros_setup_orig/. /opt/ros/$ROS2_DISTRO/ && \
    rm -rf /tmp/ros_setup_orig /ros2_ws

# clean up
RUN apt remove -y --auto-remove \
    software-properties-common build-essential gcc python3-colcon-* perl && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY scripts /scripts
COPY tests /tests

# Containers start in / (the build steps above ran in /workspace)
WORKDIR /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "spinnaker_camera_driver", "blackfly_s_gige.launch.py"]

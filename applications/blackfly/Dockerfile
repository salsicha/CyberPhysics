# Cross platform support
ARG TAG
FROM --platform=$TARGETPLATFORM cyberphysics/ros2:$TAG AS build

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
    software-properties-common build-essential libgomp1

COPY files /files

WORKDIR /files
# Accept the EULA
RUN echo 'libgentl libspinnaker/accepted-flir-eula boolean true' | debconf-set-selections && \
    tar xzvf spinnaker-3.2.0.57-${TARGETARCH}-pkg-22.04.tar.gz && \
    cd spinnaker-3.2.0.57-${TARGETARCH} && \
    apt -y install ./libgentl_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker-dev_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker-c_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinnaker-c-dev_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinvideo_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinvideo-c_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./libspinvideo-dev_3.2.0.57_${TARGETARCH}.deb && \
    apt -y install ./spinnaker_3.2.0.57_${TARGETARCH}.deb && \
    rm -rf /files

COPY src /workspace/src
WORKDIR /workspace

# enable cache
RUN --mount=type=cache,target=/var/cache/apt \
    . /opt/ros/$ROS2_DISTRO/setup.bash && \
    apt update && \
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro $ROS2_DISTRO

RUN . /opt/ros/$ROS2_DISTRO/setup.bash && \
    colcon build --merge-install --install-base /opt/ros/$ROS2_DISTRO && \
    rm -rf /ros2_ws

# clean up
RUN apt remove -y --auto-remove \
    software-properties-common build-essential gcc python3-colcon-* perl && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY scripts /scripts
COPY tests /tests

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "spinnaker_camera_driver", "blackfly_s_gige.launch.py"]

# Cross platform support

# FROM --platform=$TARGETPLATFORM cyberphysics/ros2 AS build
FROM cyberphysics/ros2 AS build

ARG TAG

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=$ROS2_DISTRO

RUN apt-get update && \
    apt-get install -y --no-install-recommends --reinstall liblog4cxx-dev systemd-sysv \
    ros-$ROS2_DISTRO-ros2bag ros-$ROS2_DISTRO-rosbag2-storage-default-plugins ros-$ROS2_DISTRO-rosbridge-msgs && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install rosbags==0.9.22

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY scripts /scripts
COPY tests /tests

RUN touch /env.list

#####################################################################
# Final release build
FROM scratch

ARG GIT_COMMIT
# GIT_COMMIT=$(git rev-parse HEAD)
ARG GIT_BRANCH
# GIT_BRANCH=$(git branch --show-current)
ARG GIT_TAG
# GIT_TAG=$(git describe --tag)

COPY --from=build / /

ENV GIT_COMMIT=${GIT_COMMIT}
ENV GIT_BRANCH=${GIT_BRANCH}
ENV GIT_TAG=${GIT_TAG}

WORKDIR /scripts

ENTRYPOINT ["/entrypoint.sh"]

CMD ["python3", "record.py", "example_system"]

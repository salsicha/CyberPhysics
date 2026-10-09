# Cross platform support
ARG TAG=latest

# FROM --platform=$TARGETPLATFORM salsicha/cyberphysics:ros2-${TAG} AS build
FROM salsicha/cyberphysics:ros2-${TAG} AS build

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

# Compose files mount the system env list directory here (see entrypoint.sh).
RUN mkdir -p /env

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

ENV PATH="/venv/bin:$PATH"

ENV GIT_COMMIT=${GIT_COMMIT}
ENV GIT_BRANCH=${GIT_BRANCH}
ENV GIT_TAG=${GIT_TAG}

WORKDIR /scripts

ENTRYPOINT ["/entrypoint.sh"]

CMD ["python3", "record.py", "--proj_name", "example_system"]

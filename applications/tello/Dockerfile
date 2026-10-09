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

WORKDIR /

# Download /tello-ros2-main/
RUN curl -L -o /main.zip https://github.com/salsicha/tello-ros2/archive/refs/heads/main.zip
RUN unzip /main.zip && rm /main.zip

# The scripts use paths relative to the scripts directory (../workspace)
WORKDIR /tello-ros2-main/scripts

RUN chmod +x ./*
RUN ./install.sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && ./build.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

WORKDIR /tello-ros2-main/scripts

ENTRYPOINT ["/entrypoint.sh"]
CMD ["./run.sh"]

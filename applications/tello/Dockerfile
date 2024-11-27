
FROM cyberphysics/ros2:latest AS build

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
RUN curl -L -k -o /main.zip https://github.com/salsicha/tello-ros2/archive/refs/heads/main.zip
RUN unzip /main.zip && rm /main.zip

WORKDIR /tello-ros2-main/

RUN chmod +x scripts/*
RUN ./scripts/install.sh
RUN ./scripts/build.sh

#####################################################################
FROM scratch

COPY --from=build / /

WORKDIR /tello-ros2-main/

ENTRYPOINT ["/entrypoint.sh"]
CMD ["./script/run.sh"]


FROM cyberphysics/ros2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN python3 -m venv /venv
RUN . /venv/bin/activate && pip3 install nicegui

# ADD ros2_ws /ros2_ws
# WORKDIR /ros2_ws
# RUN source /opt/ros/jazzy/setup.bash && \
#     colcon build --symlink-install

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh


#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

EXPOSE 8080

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "gui", "main_launch.py"]

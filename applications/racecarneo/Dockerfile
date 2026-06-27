FROM cyberphysics/ros2:latest AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS2_DISTRO=jazzy \
    RACECAR_HOME=/opt/racecarneo \
    RACECAR_SIM_DIR=/opt/racecarneo/RacecarNeo-Simulator/RacecarSim_Linux_v2.8.3 \
    RACECAR_LIBRARY_DIR=/opt/racecarneo/racecar-neo-library/library \
    RACECAR_SIM_IP=127.0.0.1 \
    PATH="/venv/bin:$PATH" \
    PYTHONPATH="/opt/racecarneo/racecar-neo-library/library:/opt/racecarneo/racecar-neo-library/library/simulation"

SHELL ["/bin/bash", "-c"]

ARG RACECAR_SIM_REF=linux
ARG RACECAR_LIBRARY_REF=main
ARG RACECAR_SIM_REPO=https://github.com/MITRacecarNeo/RacecarNeo-Simulator.git
ARG RACECAR_LIBRARY_REPO=https://github.com/MITRacecarNeo/racecar-neo-library.git

RUN apt update && apt install -q -y --no-install-recommends \
    ca-certificates \
    curl \
    ffmpeg \
    git \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libx11-6 \
    libxext6 \
    libxrandr2 \
    libxcursor1 \
    libxi6 \
    libxinerama1 \
    libxss1 \
    libxtst6 \
    mesa-utils \
    python3-opencv \
    python3-pip \
    python3-venv \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-tf2-ros \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv && \
    /venv/bin/pip install --no-cache-dir --upgrade pip && \
    /venv/bin/pip install --no-cache-dir \
      numpy==1.26.2 \
      matplotlib==3.8.4 \
      pandas==2.2.3 \
      mypy \
      nptyping==2.5.0 \
      opencv-python==4.10.0.84 \
      jinja2==3.1.4 \
      ipywidgets==8.1.5 \
      scipy==1.14.1 \
      rich==14.0.0

RUN mkdir -p ${RACECAR_HOME} && \
    git clone --depth 1 --branch ${RACECAR_SIM_REF} ${RACECAR_SIM_REPO} ${RACECAR_HOME}/RacecarNeo-Simulator && \
    git clone --depth 1 --branch ${RACECAR_LIBRARY_REF} ${RACECAR_LIBRARY_REPO} ${RACECAR_HOME}/racecar-neo-library && \
    chmod +x ${RACECAR_SIM_DIR}/RacecarSim_Linux_v2.8.3.x86_64

COPY scripts/racecarneo_ros_bridge.py /workspace/racecarneo_ros_bridge.py
COPY scripts/cmd_vel_to_ackermann.py /workspace/cmd_vel_to_ackermann.py
COPY scripts/run_nav2_mission.py /workspace/run_nav2_mission.py
COPY scripts/validate_stack_integration.py /workspace/validate_stack_integration.py
COPY scripts/run_headless_sim.sh /workspace/run_headless_sim.sh
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /workspace/racecarneo_ros_bridge.py /workspace/cmd_vel_to_ackermann.py /workspace/run_nav2_mission.py /workspace/validate_stack_integration.py /workspace/run_headless_sim.sh

FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS2_DISTRO=jazzy \
    RACECAR_HOME=/opt/racecarneo \
    RACECAR_SIM_DIR=/opt/racecarneo/RacecarNeo-Simulator/RacecarSim_Linux_v2.8.3 \
    RACECAR_LIBRARY_DIR=/opt/racecarneo/racecar-neo-library/library \
    RACECAR_SIM_IP=127.0.0.1 \
    PATH="/venv/bin:$PATH" \
    PYTHONPATH="/opt/racecarneo/racecar-neo-library/library:/opt/racecarneo/racecar-neo-library/library/simulation"

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/workspace/racecarneo_ros_bridge.py"]

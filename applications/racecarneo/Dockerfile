ARG TAG=latest
# RacecarSim v2.8.3 (UDP protocol 1) and the library commit that speaks it,
# pinned together. v2.9+ moved to protocol 2, REP-103 IMU axes and 1080
# lidar samples, which racecarneo_ros_bridge.py does not expect.
ARG RACECAR_SIM_VERSION=v2.8.3

FROM salsicha/cyberphysics:ros2-${TAG}

ARG RACECAR_SIM_VERSION
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS2_DISTRO=jazzy \
    RACECAR_HOME=/opt/racecarneo \
    RACECAR_SIM_DIR=/opt/racecarneo/RacecarNeo-Simulator/RacecarSim_Linux_${RACECAR_SIM_VERSION} \
    RACECAR_LIBRARY_DIR=/opt/racecarneo/racecar-neo-library/library \
    RACECAR_SIM_IP=127.0.0.1 \
    PATH="/venv/bin:$PATH" \
    PYTHONPATH="/opt/racecarneo/racecar-neo-library/library:/opt/racecarneo/racecar-neo-library/library/simulation"

SHELL ["/bin/bash", "-c"]

# RacecarSim v2.8.3 on the linux branch, and library main as of 2026-05-13.
ARG RACECAR_SIM_COMMIT=250f72d0c19f57161d53edaf325abf743da41a15
ARG RACECAR_LIBRARY_COMMIT=494d16e821394fc8e41a1c41a88e742e407d24ce
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

# Neither repository uses Git LFS (no .gitattributes at these commits).
RUN fetch_commit() { git init -q "$3" && git -C "$3" fetch -q --depth 1 "$1" "$2" && git -C "$3" checkout -q FETCH_HEAD; } && \
    mkdir -p ${RACECAR_HOME} && \
    fetch_commit ${RACECAR_SIM_REPO} ${RACECAR_SIM_COMMIT} ${RACECAR_HOME}/RacecarNeo-Simulator && \
    fetch_commit ${RACECAR_LIBRARY_REPO} ${RACECAR_LIBRARY_COMMIT} ${RACECAR_HOME}/racecar-neo-library && \
    chmod +x "${RACECAR_SIM_DIR}/RacecarSim_Linux_${RACECAR_SIM_VERSION}.x86_64"

COPY scripts/racecarneo_ros_bridge.py /workspace/racecarneo_ros_bridge.py
COPY scripts/cmd_vel_to_ackermann.py /workspace/cmd_vel_to_ackermann.py
COPY scripts/run_nav2_mission.py /workspace/run_nav2_mission.py
COPY scripts/validate_stack_integration.py /workspace/validate_stack_integration.py
COPY scripts/score_sensor_behavior.py /workspace/score_sensor_behavior.py
COPY scripts/racecarneo_acceptance_report.py /workspace/racecarneo_acceptance_report.py
COPY scripts/run_headless_sim.sh /workspace/run_headless_sim.sh
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /workspace/racecarneo_ros_bridge.py /workspace/cmd_vel_to_ackermann.py /workspace/run_nav2_mission.py /workspace/validate_stack_integration.py /workspace/score_sensor_behavior.py /workspace/racecarneo_acceptance_report.py /workspace/run_headless_sim.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/workspace/racecarneo_ros_bridge.py"]

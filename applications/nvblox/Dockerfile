
FROM ubuntu:jammy AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    PATH="/usr/local/cuda/bin:$PATH" \
    CUDACXX=/usr/local/cuda/bin/nvcc
    # LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# ROS2
RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv unzip ca-certificates jq tar sudo wget vim git \
    software-properties-common build-essential gcc imagemagick && \
    rm -rf /var/lib/apt/lists/*
RUN curl --insecure -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN python3 -m venv /venv

RUN pip3 uninstall em
RUN pip3 install rosdep colcon-common-extensions pytest-rerunfailures numpy lark empy==3.3.4 catkin_pkg

# Install ROS2 packages
RUN apt update && apt install -q -y --no-install-recommends \
    ros-$ROS2_DISTRO-ros-core python3-rosdep python3-colcon-ros linuxptp python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb
RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    cuda-toolkit nvidia-gds && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/src/

# Build and install nvblox
RUN git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
RUN git clone --recursive -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox

WORKDIR /workspace

RUN apt update && \
    . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    rosdep install -i -r --from-paths ./src/isaac_ros_nvblox/ --rosdistro humble -y && \
    colcon build --symlink-install --packages-up-to isaac_ros_nvblox && \
    rm -rf /var/lib/apt/lists/*

# Install VPI packages
RUN wget https://repo.download.nvidia.com/jetson/x86_64/jammy/pool/main/libn/libnvvpi3/libnvvpi3_3.2.4_amd64.deb && \
    dpkg -i libnvvpi* && rm libnvvpi*
RUN wget https://repo.download.nvidia.com/jetson/x86_64/jammy/pool/main/v/vpi3-dev/vpi3-dev_3.2.4_amd64.deb && \
    dpkg -i vpi3* && rm vpi3*

# Quickstart assets
COPY setup.sh /setup.sh
RUN chmod +x /setup.sh
RUN /setup.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
    

#####################################################################
FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch nvblox_examples_bringup", "isaac_sim_example.launch.py", "rosbag:=/workspace/isaac_ros_assets/isaac_ros_nvblox/quickstart", "navigation:=False"]

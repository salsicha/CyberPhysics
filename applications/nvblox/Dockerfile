
FROM ubuntu:jammy AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    CUDACXX=/usr/local/cuda/bin/nvcc \
    PATH="/usr/local/cuda/bin:$PATH" \
    LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH" \
    SHELL=/bin/bash

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# ROS2
RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv unzip ca-certificates jq tar sudo wget vim git \
    software-properties-common build-essential gcc imagemagick && \
    rm -rf /var/lib/apt/lists/*
RUN curl --insecure -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add Isaac apt repository
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | tee -a /etc/apt/sources.list

# Install VPI packages
RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
    add-apt-repository "deb http://repo.download.nvidia.com/jetson/x86_64/$(lsb_release -cs) r36.3 main" && \
    apt update && \
    apt install -q -y --no-install-recommends libnvvpi3 vpi3-dev

# CUDA packages
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb
RUN apt update && apt upgrade -y && apt install -q -y --no-install-recommends \
    cuda-toolkit nvidia-gds

# Install nvblox and apt packages
RUN add-apt-repository universe && \
    apt update && apt install -q -y --no-install-recommends \
    ros-$ROS2_DISTRO-ros-core ros-$ROS2_DISTRO-rviz-default-plugins \
    ros-$ROS2_DISTRO-isaac-ros-managed-nitros ros-$ROS2_DISTRO-isaac-ros-nitros-image-type \
    ros-$ROS2_DISTRO-isaac-ros-nvblox ros-$ROS2_DISTRO-rviz2 \
    ros-$ROS2_DISTRO-isaac-ros-realsense ros-$ROS2_DISTRO-as2-realsense-interface \
    ros-$ROS2_DISTRO-librealsense2 ros-$ROS2_DISTRO-realsense2-camera ros-$ROS2_DISTRO-realsense2-description \
    python3-rosdep \
    python3-colcon-ros \
    linuxptp \
    python3-colcon-common-extensions \
    libthrust-dev \  
    python3-pip \
    python3-pybind11 \
    python3-pytest \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-cov \
    apt-transport-https \
    bash-completion \
    build-essential \
    ca-certificates \
    clang-format \
    cmake \
    curl \
    git \
    git-lfs \
    gnupg2 \
    iputils-ping \
    locales \
    lsb-release \
    rsync \
    wget \
    vim \
    unzip \
    mlocate \
	libgoogle-glog-dev \
    ffmpeg \
    libasio-dev \
    libbullet-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libmnl0 \
    libmnl-dev \
    libv4l-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    libpython3.10 \
    libx264-dev \
    kmod \
    patchelf \
    python3-opencv \
    nlohmann-json3-dev \
    mosquitto \
    mosquitto-clients \
    nvv4l2 \
    sqlite3 \
    libsqlite3-dev \
    libbenchmark-dev

RUN python3 -m venv /venv
COPY requirements.txt /requirements.txt
RUN . /venv/bin/activate && \
    pip3 uninstall em && \
    pip3 install -r /requirements.txt

### This isn't working for nvblox_ros, but maybe working for other packages
# Build and install nvblox
# WORKDIR /workspace/src/
# RUN git clone -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
# RUN git clone --recursive -b release-3.1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
# WORKDIR /workspace
# RUN apt update && \
#     . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
#     rosdep init && \
#     rosdep update && \
#     rosdep install -i -r --from-paths ./src/isaac_ros_nvblox/ --rosdistro $ROS2_DISTRO -y && \
#     colcon build --symlink-install --packages-up-to isaac_ros_nvblox && \
#     rm -rf /var/lib/apt/lists/*

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

ARG TAG=latest

# ROS 2 Jazzy (Ubuntu 24.04, Python 3.12) base shared with the other apps
FROM salsicha/cyberphysics:ros2-${TAG}

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy \
    ROS2_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# OpenCV (Ultralytics) needs libGL/GLib even for headless image conversion;
# python3-serial is the Arduino bridge's serial port; xacro and the state
# publishers are run-time dependencies of the pan_tilt_ros package below.
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libgl1 libglib2.0-0 python3-serial \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher && \
    rm -rf /var/lib/apt/lists/*

# The base image's /venv (first on PATH); let it also see apt Python packages
# (python3-serial) as the depthanything image does.
RUN python3 -m venv --system-site-packages /venv

# Ultralytics YOLOE and its CLIP text tokenizer, pinned to the versions the
# Humble image resolved (all have CPython 3.12 wheels). PyPI's x86_64 Torch
# 2.14 wheels bundle CUDA 13 (needs an NVIDIA driver >= 580).
ARG ULTRALYTICS_VERSION=8.4.52
ARG TORCH_VERSION=2.14.1
ARG TORCHVISION_VERSION=0.29.1
ARG OPENCV_VERSION=5.0.0.93
ARG CLIP_COMMIT=488e81a6711eea7346872b46ea928b367da8889d
RUN . /venv/bin/activate && \
    pip3 install --no-cache-dir \
      "torch==${TORCH_VERSION}" \
      "torchvision==${TORCHVISION_VERSION}" \
      "opencv-python==${OPENCV_VERSION}" \
      "ultralytics==${ULTRALYTICS_VERSION}" \
      "clip @ git+https://github.com/ultralytics/CLIP.git@${CLIP_COMMIT}"

# pan_tilt_ros: upstream ST3215 pan/tilt nodes (adityakamath/pan_tilt_ros, which
# supports Humble and Jazzy). Its SCServo_Linux submodule is not vendored here;
# fetch the commit upstream pins and build the static library the package links
# from source (the checkout's prebuilt libSCServo.a is aarch64, and its CMake
# project also builds nanobind Python bindings from another submodule).
ARG SCSERVO_LINUX_COMMIT=79e70ff57e37103a2c7f726bf67da69a361b7be3
COPY pantiltros/ /opt/pantiltros_ws/src/pan_tilt_ros/
RUN cd /opt/pantiltros_ws/src/pan_tilt_ros && \
    git init -q include/SCServo_Linux && \
    git -C include/SCServo_Linux fetch -q --depth 1 \
      https://github.com/adityakamath/SCServo_Linux.git ${SCSERVO_LINUX_COMMIT} && \
    git -C include/SCServo_Linux checkout -q FETCH_HEAD && \
    cd include/SCServo_Linux && \
    rm -rf .git extern examples src *.a *.so setup.py && \
    for src in SMSCL SMS_STS SCSerial SCS SCSCL SMSBL; do \
      g++ -std=c++17 -O3 -fPIC -c "${src}.cpp" -o "${src}.o"; \
    done && \
    ar rcs libSCServo.a ./*.o && rm -f ./*.o && \
    cd /opt/pantiltros_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --install-base /opt/pantiltros_ws/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf /opt/pantiltros_ws/build /opt/pantiltros_ws/log

# TODO:
# https://github.com/adityakamath/pan_tilt_ros

# The RACECAR Turret's USB (UVC) turret camera driver
# (compositions/racecar_turret_hardware.yaml). A layer of its own after the
# PyTorch one, so adding it did not invalidate the multi-GB wheel layer.
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-${ROS_DISTRO}-usb-cam && \
    rm -rf /var/lib/apt/lists/*

COPY scripts/ /scripts/
RUN chmod +x /scripts/*.py

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

ARG TAG=latest

FROM salsicha/cyberphysics:ros2-${TAG}

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=offscreen

SHELL ["/bin/bash", "-c"]
WORKDIR /workspace

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    git \
    python3-pip \
    python3-venv \
    libgl1 \
    libglib2.0-0 \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /venv

# arm64 gets PyPI's CPU-only wheels; Jetson GPUs need NVIDIA's JetPack builds.
ARG TARGETARCH
ARG TORCH_VERSION=2.5.1
ARG TORCHVISION_VERSION=0.20.1
RUN . /venv/bin/activate && \
    TORCH_PKGS="torch==${TORCH_VERSION} torchvision==${TORCHVISION_VERSION} torchaudio==${TORCH_VERSION}" && \
    if [ "$TARGETARCH" = "arm64" ]; then \
        pip install --no-cache-dir ${TORCH_PKGS}; \
    else \
        pip install --no-cache-dir ${TORCH_PKGS} \
            --index-url https://download.pytorch.org/whl/cu121; \
    fi && \
    pip install --no-cache-dir \
        numpy==1.26.4 \
        opencv-python-headless==4.11.0.86 \
        matplotlib==3.11.0

# Depth-Anything-V2 main as of 2026-03-24 and the model repo revision of the
# metric VKITTI small checkpoint, checked by hash.
ARG DEPTH_ANYTHING_COMMIT=a561b849ebae10a6f5ef49e26c83cbbcd36c71bf
ARG DEPTH_ANYTHING_WEIGHTS_REVISION=c725b8589bdf6ab04072cab74c0467830db80d6d
ARG DEPTH_ANYTHING_WEIGHTS_SHA256=9203e538d35255c90dda4b7fedb47ff33fe725497bcca3b1e53b3a65ee63f0cb

RUN git init -q /opt/Depth-Anything-V2 && \
    git -C /opt/Depth-Anything-V2 fetch -q --depth 1 \
      https://github.com/DepthAnything/Depth-Anything-V2.git ${DEPTH_ANYTHING_COMMIT} && \
    git -C /opt/Depth-Anything-V2 checkout -q FETCH_HEAD

RUN mkdir -p /opt/Depth-Anything-V2/checkpoints && \
    wget -O /opt/Depth-Anything-V2/checkpoints/depth_anything_v2_metric_vkitti_vits.pth \
      https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Small/resolve/${DEPTH_ANYTHING_WEIGHTS_REVISION}/depth_anything_v2_metric_vkitti_vits.pth && \
    echo "${DEPTH_ANYTHING_WEIGHTS_SHA256}  /opt/Depth-Anything-V2/checkpoints/depth_anything_v2_metric_vkitti_vits.pth" \
      | sha256sum -c -

ENV PYTHONPATH="/opt/Depth-Anything-V2/metric_depth:/opt/Depth-Anything-V2"

RUN rm -rf /workspace/src /workspace/build /workspace/install /workspace/log
COPY src /workspace/src

RUN . /venv/bin/activate && . /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build --merge-install --install-base /workspace/install && \
    rm -rf /var/lib/apt/lists/*

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Containers start in / (the build steps above ran in /workspace)
WORKDIR /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "depthanything_pkg", "depthanything.launch.py"]

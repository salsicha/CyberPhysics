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

# JetPack 7.2 (Orin, Ubuntu 24.04) supports the upstream CUDA 13 arm64
# wheels. Never let this branch resolve PyPI's CPU-only arm64 wheels.
ARG TARGETARCH
ARG TORCH_VERSION=2.5.1
ARG TORCHVISION_VERSION=0.20.1
RUN . /venv/bin/activate && \
    case "${TARGETARCH}" in \
      arm64) pip install --no-cache-dir --only-binary=:all: \
        torch==2.10.0+cu130 torchvision==0.25.0+cu130 \
        --index-url https://download.pytorch.org/whl/cu130 ;; \
      amd64) pip install --no-cache-dir \
        "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}" "torchaudio==${TORCH_VERSION}" \
        --index-url https://download.pytorch.org/whl/cu121 ;; \
      *) echo "Unsupported architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    pip install --no-cache-dir \
        numpy==1.26.4 \
        opencv-python-headless==4.11.0.86 \
        matplotlib==3.11.0

# These assertions do not need a GPU at build time, and catch CPU wheels
# and mismatched torchvision native extensions on native arm64 CI runners.
RUN python3 -c 'import torch, torchvision; assert torch.version.cuda; assert torchvision.extension._has_ops()'

# Depth-Anything-V2 main as of 2026-03-24 and the model repo revision of the
# metric VKITTI (outdoor driving) small checkpoint, checked by hash.
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

# The metric Hypersim (indoor) small checkpoint the same way: metric_dataset
# hypersim with max_depth 20 (README.md, "Accuracy").
ARG DEPTH_ANYTHING_HYPERSIM_REVISION=3bc65d4e14a6786a61acec16453c50e12bf5f338
ARG DEPTH_ANYTHING_HYPERSIM_SHA256=b782898d8a3e8be1f639de33837ed85e9b4b73e40f8f5e5cd99067588d722545
RUN wget -O /opt/Depth-Anything-V2/checkpoints/depth_anything_v2_metric_hypersim_vits.pth \
      https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/${DEPTH_ANYTHING_HYPERSIM_REVISION}/depth_anything_v2_metric_hypersim_vits.pth && \
    echo "${DEPTH_ANYTHING_HYPERSIM_SHA256}  /opt/Depth-Anything-V2/checkpoints/depth_anything_v2_metric_hypersim_vits.pth" \
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

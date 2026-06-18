ARG TAG=amd64

FROM cyberphysics/ros2:${TAG} AS build

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

ARG TARGETARCH
RUN . /venv/bin/activate && \
    if [ "$TARGETARCH" = "arm64" ]; then \
        pip install --no-cache-dir torch torchvision torchaudio; \
    else \
        pip install --no-cache-dir torch torchvision torchaudio \
            --index-url https://download.pytorch.org/whl/cu121; \
    fi && \
    pip install --no-cache-dir \
        "numpy<2" \
        opencv-python-headless \
        matplotlib

RUN git clone https://github.com/DepthAnything/Depth-Anything-V2.git /opt/Depth-Anything-V2

RUN mkdir -p /opt/Depth-Anything-V2/checkpoints && \
    wget -O /opt/Depth-Anything-V2/checkpoints/depth_anything_v2_metric_vkitti_vits.pth \
      https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Small/resolve/main/depth_anything_v2_metric_vkitti_vits.pth

ENV PYTHONPATH="/opt/Depth-Anything-V2/metric_depth:/opt/Depth-Anything-V2"

RUN rm -rf /workspace/src /workspace/build /workspace/install /workspace/log
COPY src /workspace/src

RUN . /venv/bin/activate && . /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build --merge-install --install-base /workspace/install

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

FROM scratch
COPY --from=build / /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "depthanything_pkg", "depthanything.launch.py"]

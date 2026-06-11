ARG TAG=amd64

FROM cyberphysics/aerostack2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=xcb \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace

# Install dependencies for Depth Anything V2
RUN apt-get update && apt-get install -y \
    wget \
    git \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    ros-$ROS_DISTRO-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install python requirements
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
        opencv-python \
        matplotlib

# Clone Depth Anything V2
RUN git clone https://github.com/DepthAnything/Depth-Anything-V2.git /opt/Depth-Anything-V2

# Download weights (vits)
RUN mkdir -p /opt/Depth-Anything-V2/checkpoints && \
    wget -O /opt/Depth-Anything-V2/checkpoints/depth_anything_v2_vits.pth https://huggingface.co/depth-anything/Depth-Anything-V2-Small/resolve/main/depth_anything_v2_vits.pth

ENV PYTHONPATH="/opt/Depth-Anything-V2:$PYTHONPATH"

# Copy source code
COPY src /workspace/src

# Micro-ROS agent and package build
RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    apt-get update && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build --merge-install --install-base /workspace/install

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

ARG TAG=latest

FROM salsicha/cyberphysics:ros2-${TAG}

ARG TARGETARCH

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ISAAC_ROS_RELEASE_MAJOR=4 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video \
    PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/src/tensorrt/bin:${PATH} \
    DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    ca-certificates curl gnupg jq tar \
    && rm -rf /var/lib/apt/lists/*

# Orin needs Isaac ROS 4.6 / JetPack 7.2, not the Thor-only 4.4 arm64
# packages. Keep the existing desktop stack on 4.4 / CUDA 13.0.
RUN case "${TARGETARCH}" in \
      amd64) CUDA_ARCH=x86_64; JETSON_REPO=x86_64/noble; L4T=r38.4; ISAAC_RELEASE=4.4; ISAAC_SUITE=noble ;; \
      arm64) CUDA_ARCH=sbsa; JETSON_REPO=common; L4T=r39.2; ISAAC_RELEASE=4.6; ISAAC_SUITE=noble-jetpack ;; \
      *) echo "Unsupported architecture: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    curl -fsSL "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/${CUDA_ARCH}/cuda-keyring_1.1-1_all.deb" \
      -o /tmp/cuda-keyring.deb && \
    dpkg -i /tmp/cuda-keyring.deb && \
    rm /tmp/cuda-keyring.deb && \
    curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-jetson.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-jetson.gpg] https://repo.download.nvidia.com/jetson/${JETSON_REPO} ${L4T} main" \
      > /etc/apt/sources.list.d/nvidia-jetson.list && \
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key \
      | gpg --dearmor -o /usr/share/keyrings/isaac-ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/isaac-ros-archive-keyring.gpg] https://isaac.download.nvidia.com/isaac-ros/release-${ISAAC_RELEASE} ${ISAAC_SUITE} main" \
      > /etc/apt/sources.list.d/isaac-ros.list

RUN apt update && apt install -q -y --no-install-recommends \
    ros-${ROS2_DISTRO}-isaac-ros-nvblox \
    ros-${ROS2_DISTRO}-nav2-bringup \
    ros-${ROS2_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

COPY setup.sh /setup.sh
# Recorded desktop quickstart bags are not needed on the car. Set this
# explicitly to 1 to include them in an arm64 image as well.
ARG DOWNLOAD_EXAMPLE_ASSETS=auto
RUN chmod +x /setup.sh && \
    if [ "${DOWNLOAD_EXAMPLE_ASSETS}" = 1 ] || \
       { [ "${DOWNLOAD_EXAMPLE_ASSETS}" = auto ] && [ "${TARGETARCH}" = amd64 ]; }; then /setup.sh; fi

COPY entrypoint.sh /entrypoint.sh
COPY racecarneo_nvblox.launch.py /workspace/racecarneo_nvblox.launch.py
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/workspace/racecarneo_nvblox.launch.py"]

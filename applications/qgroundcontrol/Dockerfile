## QGroundControl (pinned official release) + MAVROS for the ROS 2 bridge

ARG TAG=latest

FROM salsicha/cyberphysics:ros2-${TAG} AS build

# Official release AppImage, see https://github.com/mavlink/qgroundcontrol/releases
ARG QGC_VERSION=v5.1.5
ARG TARGETARCH

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash","-c"]

# QGroundControl runtime dependencies (QGC Ubuntu install guide), Qt xcb
# platform plugin libraries, and MAVROS for MAVLink <-> ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        espeak-ng \
        gstreamer1.0-gl \
        gstreamer1.0-libav \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-good \
        libdbus-1-3 \
        libegl1 \
        libfontconfig1 \
        libfuse2t64 \
        libgl1 \
        libxcb-cursor0 \
        libxcb-icccm4 \
        libxcb-image0 \
        libxcb-keysyms1 \
        libxcb-randr0 \
        libxcb-render-util0 \
        libxcb-shape0 \
        libxcb-xinerama0 \
        libxcb-xkb1 \
        libxkbcommon-x11-0 \
        speech-dispatcher \
        geographiclib-tools \
        ros-${ROS_DISTRO}-mavros \
        ros-${ROS_DISTRO}-mavros-extras \
        ros-${ROS_DISTRO}-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# Extract the AppImage so QGroundControl runs without FUSE inside the container
RUN case "${TARGETARCH:-amd64}" in \
        amd64) qgc_arch=x86_64 ;; \
        arm64) qgc_arch=aarch64 ;; \
        *) echo "Unsupported architecture: ${TARGETARCH}" && exit 1 ;; \
    esac && \
    curl -fsSL -o /tmp/QGroundControl.AppImage \
        "https://github.com/mavlink/qgroundcontrol/releases/download/${QGC_VERSION}/QGroundControl-${qgc_arch}.AppImage" && \
    chmod +x /tmp/QGroundControl.AppImage && \
    cd /opt && /tmp/QGroundControl.AppImage --appimage-extract > /dev/null && \
    mv /opt/squashfs-root /opt/qgroundcontrol && \
    chmod -R a+rX /opt/qgroundcontrol && \
    ln -s /opt/qgroundcontrol/AppRun /usr/local/bin/QGroundControl && \
    rm /tmp/QGroundControl.AppImage

# QGroundControl refuses to run as root; use the image's uid 1000 user
# ("ubuntu"), with serial port access for telemetry radios
RUN usermod -a -G dialout ubuntu

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

USER ubuntu

WORKDIR /home/ubuntu

ENTRYPOINT [ "/entrypoint.sh" ]

CMD [ "QGroundControl" ]

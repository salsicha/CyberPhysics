
FROM ros:humble-ros-base AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=humble

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim wget && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv


# TODO:
# https://github.com/salsicha/bcflight

# Flight Controller:
# Dependencies
# sudo apt update
# sudo apt install git pkg-config cmake make g++ libc6-dev libraspberrypi-dev libiw-dev libdrm-dev libgbm-dev libcamera-dev libgles2-mesa-dev libgps-dev libasound2-dev libcrypt-dev zlib1g-dev libpng-dev libshine-dev libavformat-dev libavutil-dev libavcodec-dev libpigpio-dev lua5.3 libfftw3-dev
# Build
# git clone https://github.com/salsicha/bcflight
# cd bcflight/flight
# cmake -Dboard=rpi -Ddebug=1 -S . -B build
# cd build && make -j$(nproc)


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"    

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]


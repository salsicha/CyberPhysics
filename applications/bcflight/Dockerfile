
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

# Controller:
# Dependencies
# sudo apt-get install nasm qtmultimedia5-dev qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libnl-3-dev libnl-genl-3-dev libnl-route-3-dev libiw-dev libfftw3-dev libqscintilla2-qt5-dev
# git clone https://github.com/salsicha/bcflight.git
# cd bcflight/controller_pc
# cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
# cd build && make -j$(nproc)
# ./controller_pc


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


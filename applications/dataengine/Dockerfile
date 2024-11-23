FROM cyberphysics/ros2:latest as build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        # pyROS packages
        python3-image-geometry x11-apps \
        # psycopg2 dependencies
        libpq-dev build-essential \
        python3-tk mlocate \
        ros-$ROS_DISTRO-cv-bridge \
        libgl1 libgomp1 libegl1 \
        xorg-dev libxcb-shm0 libglu1-mesa-dev python3-dev clang \
        libc++-dev libc++abi-dev libsdl2-dev ninja-build libxi-dev \
        libtbb-dev libosmesa6-dev libudev-dev autoconf libtool \
    && rm -rf /var/lib/apt/lists/* \
    && ln -s /usr/bin/python3 /usr/bin/python

RUN mkdir /dataengine

COPY requirements.txt /dataengine/requirements.txt
RUN pip3 install -r /dataengine/requirements.txt

# Clean up
RUN apt remove -y --auto-remove build-essential gcc && \
    rm -rf /root/.cache

COPY dataengine /dataengine/dataengine
COPY setup.py /dataengine/setup.py

# TODO: build package

#####################################################################
FROM scratch

COPY --from=build / /

WORKDIR /dataengine/

ENTRYPOINT ["/entrypoint.sh"]

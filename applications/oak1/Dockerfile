FROM cyberphysics/ros2:latest

COPY requirements.txt /

RUN apt update && apt install -y --no-install-recommends \
    python3-rosdep python3-colcon-ros python3-pip ros-humble-cv-bridge \
    && apt install -y x11-apps \
    && pip install -r /requirements.txt \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get upgrade -y

COPY scripts/driver.py /
COPY scripts/custom_script.py /
WORKDIR /

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
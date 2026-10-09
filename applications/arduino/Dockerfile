
FROM ubuntu:noble AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy \
    ARDUINO_LIBRARY_ENABLE_UNSAFE_INSTALL=true

SHELL ["/bin/bash","-c"]

## Install required dependencies for the intermediate container
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl unzip ca-certificates libfontconfig-dev libx11-dev libxft-dev \
        software-properties-common vim \
        python3-pip python3-venv python3-serial \
        git curl wget cmake build-essential \
        libx11-6 \
        libxext-dev \
        libxrender-dev \
        libxtst-dev \
        libcanberra-gtk-module \
        libxshmfence1 libglu1 \
        libnss3-dev libgdk-pixbuf2.0-dev libgtk-3-dev libxss-dev libsecret-1-dev \
        libcanberra-gtk3-module x11-apps && \
        rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

COPY scripts /home/ubuntu/scripts

## Arduino IDE, pinned (checksum from the GitHub release asset)
ARG ARDUINO_IDE_VERSION=2.3.10
ARG ARDUINO_IDE_SHA256=cc8a0b01e763d4646b670ce70c1bc8c389a0fa14ab556dcc0749c03f475a7975
RUN curl -fL -o /arduino.zip \
      "https://github.com/arduino/arduino-ide/releases/download/${ARDUINO_IDE_VERSION}/arduino-ide_${ARDUINO_IDE_VERSION}_Linux_64bit.zip" && \
    echo "${ARDUINO_IDE_SHA256}  /arduino.zip" | sha256sum -c -
## Unzip the Arduino Pro IDE
RUN unzip /arduino.zip -d /opt && rm /arduino.zip
RUN mv /opt/arduino* /opt/arduino-ide ||:

WORKDIR /home/ubuntu/

## Arduino CLI:
## TODO: Upgrade to jazzy when available
## TODO: this doesn't work, scripts can't find micro-ros
# RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh
# RUN /usr/local/bin/arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
# RUN /usr/local/bin/arduino-cli core install teensy:avr
# RUN /usr/local/bin/arduino-cli lib install micro_ros_arduino
# RUN find . -name *.ino | xargs -I {} bash -c "/usr/local/bin/arduino-cli compile {} --fqbn teensy:avr:teensy41 -v" || true

## PlatformIO
WORKDIR /home/ubuntu/scripts
ARG PLATFORMIO_VERSION=6.2.0
RUN . /venv/bin/activate && pip3 install "platformio==${PLATFORMIO_VERSION}"

## Hack to fix pio trying to enforce its own python environment
RUN mkdir -p /root/.platformio/penv/bin/
RUN cp /venv/bin/activate /root/.platformio/penv/bin/activate

## Install all projects
RUN . /venv/bin/activate && bash -c 'for i in ./*/; do (cd "$i" && pio lib install && pio run) || exit 1; done'
## Debug:
# RUN . /venv/bin/activate && cd example && bash -c 'pio lib install && pio run'

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    DISPLAY=:0

COPY --from=build / /

WORKDIR /home/ubuntu/scripts

ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/opt/arduino-ide/arduino-ide","--no-sandbox"]

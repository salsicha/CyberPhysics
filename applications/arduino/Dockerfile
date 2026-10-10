
FROM ubuntu:noble

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

## Arduino IDE, pinned (checksum from the GitHub release asset). Download,
## unzip and delete the zip in one step so it is not kept in a layer.
ARG ARDUINO_IDE_VERSION=2.3.10
ARG ARDUINO_IDE_SHA256=cc8a0b01e763d4646b670ce70c1bc8c389a0fa14ab556dcc0749c03f475a7975
RUN curl -fL -o /arduino.zip \
      "https://github.com/arduino/arduino-ide/releases/download/${ARDUINO_IDE_VERSION}/arduino-ide_${ARDUINO_IDE_VERSION}_Linux_64bit.zip" && \
    echo "${ARDUINO_IDE_SHA256}  /arduino.zip" | sha256sum -c - && \
    unzip /arduino.zip -d /opt && rm /arduino.zip && \
    { mv /opt/arduino* /opt/arduino-ide ||:; }

WORKDIR /home/ubuntu/

## micro_ros_arduino on a Teensy with arduino-cli (not used; see below for the
## arduino-cli that builds the UNO R4 / UNO Q sketches):
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

## Install all PlatformIO projects (the folders with a platformio.ini; the
## others are arduino-cli sketches, built below)
RUN . /venv/bin/activate && bash -c 'for i in */platformio.ini; do (cd "$(dirname "$i")" && pio lib install && pio run) || exit 1; done'
## Debug:
# RUN . /venv/bin/activate && cd example && bash -c 'pio lib install && pio run'

## arduino-cli with the UNO R4 and UNO Q board packages, pinned, for the
## sketches PlatformIO cannot build (it does not support the UNO Q MCU:
## platformio/platform-ststm32#869): turret_firmware (UNO R4 WiFi and UNO Q)
## and unoq_sub (UNO Q). Arduino IDE 2 in this image uses the same
## /root/.arduino15 and /root/Arduino, so it sees these cores and libraries.
## The UNO Q core brings the Zephyr toolchain (~0.8 GB) and the UNO R4 core
## GCC 7 (~0.4 GB); the download cache and build outputs are removed.
ARG ARDUINO_CLI_VERSION=1.5.1
ARG ARDUINO_CLI_SHA256=28a8e119c498a25607821c36cb2dc49e8463941b261a0d99091baa7bc692dd2b
ARG ARDUINO_RENESAS_UNO_VERSION=1.6.0
ARG ARDUINO_ZEPHYR_VERSION=1.0.0
## Arduino_RouterBridge (UNO Q MCU <-> Linux RPC) and its dependencies,
## installed with --no-deps so every version is the pinned one
ARG ARDUINO_LIBRARIES="Arduino_RouterBridge@0.4.3 Arduino_RPClite@0.3.1 MsgPack@0.4.2 DebugLog@0.8.4 ArxContainer@0.7.0 ArxTypeTraits@0.3.2"
RUN curl -fL -o /tmp/arduino-cli.tar.gz \
      "https://github.com/arduino/arduino-cli/releases/download/v${ARDUINO_CLI_VERSION}/arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz" && \
    echo "${ARDUINO_CLI_SHA256}  /tmp/arduino-cli.tar.gz" | sha256sum -c - && \
    tar -xzf /tmp/arduino-cli.tar.gz -C /usr/local/bin arduino-cli && \
    rm /tmp/arduino-cli.tar.gz && \
    arduino-cli core update-index && \
    arduino-cli core install "arduino:renesas_uno@${ARDUINO_RENESAS_UNO_VERSION}" && \
    arduino-cli core install "arduino:zephyr@${ARDUINO_ZEPHYR_VERSION}" && \
    arduino-cli lib update-index && \
    for lib in ${ARDUINO_LIBRARIES}; do arduino-cli lib install --no-deps "${lib}" || exit 1; done && \
    rm -rf /root/.arduino15/staging

## Compile every arduino-cli sketch for its boards (fails the build on errors)
RUN for target in \
      "turret_firmware arduino:renesas_uno:unor4wifi" \
      "turret_firmware arduino:zephyr:unoq" \
      "unoq_sub arduino:zephyr:unoq"; do \
      set -- ${target}; \
      echo ">>> $1 for $2" && \
      arduino-cli compile --fqbn "$2" --build-path "/tmp/arduino-build/$1" "$1" || exit 1; \
      rm -rf /tmp/arduino-build; \
    done && \
    rm -rf /root/.cache/arduino /tmp/arduino /tmp/arduino-build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV DISPLAY=:0

WORKDIR /home/ubuntu/scripts

ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/opt/arduino-ide/arduino-ide","--no-sandbox"]

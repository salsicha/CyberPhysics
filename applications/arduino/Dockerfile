
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

## Arduino IDE
## Fetch the latest Arduino Pro IDE release version
RUN curl -L -o /arduino.zip $(curl --silent https://api.github.com/repos/arduino/arduino-ide/releases/latest | grep "browser_download_url.*Linux_64bit.zip" | cut -d : -f 2,3 | xargs)
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
RUN . /venv/bin/activate && pip3 install platformio

## Hack to fix pio trying to enforce its own python environment
RUN mkdir -p /root/.platformio/penv/bin/
RUN cp /venv/bin/activate /root/.platformio/penv/bin/activate

## Install all projects
RUN . /venv/bin/activate && bash -c 'for i in ./*; do cd "$i" && pio lib install && pio run && cd -; done'
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

WORKDIR /

ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/opt/arduino-ide/arduino-ide","--no-sandbox"]

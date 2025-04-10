
# FROM ubuntu:noble AS build

# Ubuntu Jammy (22.04) goes with ROS Iron

FROM ubuntu:jammy AS build

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

# Install required dependencies for the intermediate container
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl unzip ca-certificates libfontconfig-dev libx11-dev libxft-dev \
        software-properties-common \
        python3-pip python3-venv python3-serial \
        git curl wget \
        libx11-6 \
        libxext-dev \
        libxrender-dev \
        libxtst-dev \
        libcanberra-gtk-module \
        libxshmfence1 libglu1 \
        libnss3-dev libgdk-pixbuf2.0-dev libgtk-3-dev libxss-dev libsecret-1-dev \
        libcanberra-gtk3-module x11-apps && \
        rm -rf /var/lib/apt/lists/*

# Fetch the latest Arduino Pro IDE release version
RUN curl -L -o /arduino.zip $(curl --silent https://api.github.com/repos/arduino/arduino-ide/releases/latest | grep "browser_download_url.*Linux_64bit.zip" | cut -d : -f 2,3 | xargs)

# Unzip the Arduino Pro IDE
RUN unzip /arduino.zip -d /opt && rm /arduino.zip
RUN mv /opt/arduino* /opt/arduino-ide ||:

WORKDIR /home/ubuntu/

COPY scripts /home/ubuntu/scripts

# TODO: Upgrade to jazzy when available
RUN curl -L -o /microrosarduino.zip https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.7-iron.zip
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/usr/local/bin sh
RUN /usr/local/bin/arduino-cli lib install --zip-path /microrosarduino.zip
RUN /usr/local/bin/arduino-cli core update-index --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
RUN /usr/local/bin/arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
RUN /usr/local/bin/arduino-cli core install teensy:avr
RUN find . -name *.ino | xargs -I {} bash -c "/usr/local/bin/arduino-cli compile {} --fqbn teensy:avr:teensy41 -v" || true

# I installed teensy, not teensyduino?
# https://www.pjrc.com/teensy/td_159/TeensyduinoInstall.linux64

# chmod 755 TeensyduinoInstall.linux64
# ./TeensyduinoInstall.linux64

# for me it was $ export ARDUINO_PATH=/home/manzur/arduino-1.8.13/
# export ARDUINO_PATH=[Your Arduino + Teensiduino path]

# cd $ARDUINO_PATH/hardware/teensy/avr/

# curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/foxy/extras/patching_boards/platform_teensy.txt > platform.txt

# /root/Arduino/libraries/micro_ros_arduino





# Need build flag???
# arduino-cli compile HardwareTest.ino --output-dir Build/ --fqbn teensy:avr:teensy41 -v

# Udev???
# https://www.pjrc.com/teensy/00-teensy.rules
# copy the file to /etc/udev/rules.d.
# sudo cp 00-teensy.rules /etc/udev/rules.d/


RUN python3 -m venv /venv

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

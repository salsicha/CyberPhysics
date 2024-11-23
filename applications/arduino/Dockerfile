
FROM microros/micro-ros-agent:jazzy AS build

# docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev [YOUR BOARD PORT] -v6

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash","-c"]

# Install required dependencies for the intermediate container
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl unzip ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Fetch the latest Arduino Pro IDE release version
RUN curl -L -o /arduino.zip $(curl --silent https://api.github.com/repos/arduino/arduino-ide/releases/latest | grep "browser_download_url.*Linux_64bit.zip" | cut -d : -f 2,3 | xargs)

# Unzip the Arduino Pro IDE
RUN unzip /arduino.zip -d /opt && rm /arduino.zip
RUN mv /opt/arduino* /opt/arduino-ide

COPY scripts /scripts

WORKDIR /root

RUN curl -L -o microrosarduino.zip https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.7-iron.zip

# Install required dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        software-properties-common \
        python3-pip python3-venv python3-serial \
        git \
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

WORKDIR /root

ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["/opt/arduino-ide/arduino-ide","--no-sandbox"]

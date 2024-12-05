## QGroundControl

FROM cyberphysics/ros2:latest

ARG QT_VERSION=5.15.2

ENV DEBIAN_FRONTEND noninteractive

ENV DISPLAY :99

ENV QMAKESPEC linux-g++-64

ENV QT_PATH /opt/Qt
ENV QT_DESKTOP $QT_PATH/${QT_VERSION}/gcc_64

ENV PATH /usr/lib/ccache:$QT_DESKTOP/bin:$PATH

# COPY qgroundcontrol /qgroundcontrol
RUN curl -L -k -o /master.zip https://github.com/mavlink/qgroundcontrol/archive/refs/heads/master.zip && \
    unzip /master.zip && rm /master.zip && mv /qgroundcontrol-master/ /qgroundcontrol
RUN chmod +rwx /qgroundcontrol

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
		apt-utils \
		binutils \
		build-essential \
		ca-certificates \
		ccache \
		checkinstall \
		cmake \
		curl \
		espeak \
		fuse \
		g++ \
		gcc \
		git \
		gosu \
		kmod \
		libespeak-dev \
		libfontconfig1 \
		libfuse2 \
		libgstreamer-plugins-base1.0-dev \
		libgstreamer1.0-0 \
		libgstreamer1.0-dev \
		libsdl2-dev \
		libssl-dev \
		libudev-dev \
		locales \
		make \
		ninja-build \
		openssh-client \
		openssl \
		patchelf \
		pkg-config \
		rsync \
		speech-dispatcher \
		wget \
		xvfb \
		zlib1g-dev \
		ros-humble-mavros \
		ros-humble-mavros-extras \
		ros-humble-mavros-msgs \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x /install_geographiclib_datasets.sh
RUN /install_geographiclib_datasets.sh

# Install Qt
COPY /install-qt-linux.sh /tmp/qt/
RUN /tmp/qt/install-qt-linux.sh

# Reconfigure locale
RUN locale-gen en_US.UTF-8 && dpkg-reconfigure locales

RUN mkdir /qgroundcontrol/build

WORKDIR /qgroundcontrol/build

RUN qmake ../
RUN make -j4

RUN apt install -y libxcb-icccm4 \
	libxcb-image0 \
	libxcb-util1 \
	libxcb-keysyms1 \
	libxcb-randr0 \
	libxcb-render-util0 \
	libxcb-render0 \
	libxcb-shape0 \
	libxcb-xinerama0 \
	libxcb-xkb1 \
	libxkbcommon-x11-0

# create user with id 1000 to not run commands/generate files as root
RUN useradd user --create-home --home-dir /home/user --shell /bin/bash --uid 1000
USER user

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

CMD /bin/bash

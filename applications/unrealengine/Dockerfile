
## Unreal Engine

ARG TAG=latest

FROM salsicha/cyberphysics:ros2-${TAG}

## Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

## Enable Vulkan support
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache
RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt \
	apt-get update && apt-get install -y --no-install-recommends libvulkan1 && \
	VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` && \
	mkdir -p /etc/vulkan/icd.d/ && \
	echo \
	"{\
		\"file_format_version\" : \"1.0.0\",\
		\"ICD\": {\
			\"library_path\": \"libGLX_nvidia.so.0\",\
			\"api_version\" : \"${VULKAN_API_VERSION}\"\
		}\
	}" > /etc/vulkan/icd.d/nvidia_icd.json

## Install the `xdg-user-dir` tool so the Unreal Engine can use it to locate the user's Documents directory
RUN --mount=type=cache,target=/var/cache/apt --mount=type=cache,target=/var/lib/apt \
	apt-get update && apt-get install -y --no-install-recommends xdg-user-dirs sudo x11-xserver-utils wget vim

## rosbridge_server (the ROSIntegration path) runs on the /venv python, which lacks
## the cbor2 module that rosbridge_library imports (the apt python3-cbor2, same
## version, is only visible to the system python)
RUN pip3 install --no-cache-dir cbor2==5.6.2

## ROSIntegration always talks BSON, sent as binary websocket frames, but
## rosbridge_server 2.x decodes every binary frame as UTF-8 text first, so
## bson_only_mode never parses them (RobotWebTools/rosbridge_suite#718). Skip the
## decode in BSON-only mode; fail the build if the code it patches changes.
RUN f=/opt/ros/jazzy/lib/python3.12/site-packages/rosbridge_server/websocket_handler.py && \
	[ "$(grep -c '^        if isinstance(message, bytes):$' "$f")" = 1 ] && \
	sed -i 's/^        if isinstance(message, bytes):$/        if isinstance(message, bytes) and not self.protocol.bson_only_mode:/' "$f" && \
	grep -q 'if isinstance(message, bytes) and not self.protocol.bson_only_mode:' "$f"

## The server launcher and the client example. The packaged UE build itself is
## bind-mounted at /server/build at run time (.dockerignore keeps it out).
COPY client /client
COPY server /server

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

## Create the non-root user (replacing Ubuntu 24.04's default uid 1000 "ubuntu" user)
ARG USERNAME=unrealuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G audio,video $USERNAME
USER $USERNAME

RUN sudo chown -R unrealuser:unrealuser /server /client
RUN sudo chmod +x /server/run_server.sh /client/example.py

## NVIDIA driver capabilities the container toolkit mounts at run time:
## - graphics: the Vulkan/GLX driver UE renders with (the ICD file above points
##   at libGLX_nvidia.so.0)
## - compute: some UE versions unconditionally assume `libcuda.so.1` exists with
##   the NVIDIA proprietary drivers and fail to initialise the Vulkan RHI without it
## - display: on-screen rendering through the host's X11 socket (or VirtualGL)
## - video: NVENC for plugins that use it (e.g. Pixel Streaming)
## - utility: nvidia-smi, which the launcher uses to report the GPU
ENV NVIDIA_DRIVER_CAPABILITIES=compute,display,graphics,utility,video

## Unreal Engine refuses to run as root
USER unrealuser
WORKDIR /server

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/server/run_server.sh"]


# Pinned Space ROS release; the demos checkout below uses the matching tag.
ARG SPACEROS_RELEASE=jazzy-2026.07.0
FROM osrf/space-ros:${SPACEROS_RELEASE} AS build
ARG SPACEROS_RELEASE

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim wget && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv

# Demos
RUN git clone --depth 1 --branch "${SPACEROS_RELEASE}" https://github.com/space-ros/demos.git /demos
WORKDIR /demos/curiosity_rover
RUN ./build.sh

COPY scripts/ /scripts/
RUN chmod +x /scripts/*.py

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

CMD [ "/demos/curiosity_rover/run.sh" ]

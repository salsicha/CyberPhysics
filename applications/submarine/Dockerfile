ARG TAG=latest

FROM salsicha/cyberphysics:aerostack2-${TAG}

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=jazzy \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

COPY scripts/ /scripts/
RUN chmod +x /scripts/*.py

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Containers start in / (the base image's working directory is /workspace)
WORKDIR /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["mavros_mirror"]

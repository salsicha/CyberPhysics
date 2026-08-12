
FROM ros:humble-ros-base AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=humble

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt update && apt install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-serial python3-venv vim wget && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /venv

RUN . /venv/bin/activate && \
    pip3 install --no-cache-dir \
      ultralytics==8.4.52 \
      "git+https://github.com/ultralytics/CLIP.git@488e81a6711eea7346872b46ea928b367da8889d"

# The ROS Humble base's old pip misreads CLIP's PEP 621 metadata as UNKNOWN.
# Upgrade packaging tools after the large, cacheable Torch layer and reinstall
# the small text encoder with an explicit package name. OpenCV needs libGL even
# for the headless ROS image conversion used by this node.
RUN apt update && apt install -q -y --no-install-recommends \
    libgl1 libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/pip \
    . /venv/bin/activate && \
    pip3 install --upgrade "pip<26" "setuptools>=77,<80" wheel && \
    pip3 install ftfy regex tqdm && \
    pip3 install --no-deps --force-reinstall \
      "clip @ git+https://github.com/ultralytics/CLIP.git@488e81a6711eea7346872b46ea928b367da8889d"

# TODO:
# https://github.com/adityakamath/pan_tilt_ros

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

ARG TAG=amd64

FROM cyberphysics/aerostack2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    QT_QPA_PLATFORM=xcb \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

ARG SEED_NAV_ASSETS=1
ARG NAV_ASSET_LAT=37.7694
ARG NAV_ASSET_LON=-122.4862
ARG DEMNAV_AREA_KM=10.0
ARG DEMNAV_DEM_RESOLUTION_M=30.0

WORKDIR /workspace

RUN apt update && apt install -q -y --no-install-recommends \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/pip \
    . /venv/bin/activate && \
    pip install --timeout 300 --retries 10 "srtm.py"

COPY src /workspace/src

RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    colcon build --merge-install --install-base /workspace/install

RUN if [ "$SEED_NAV_ASSETS" = "1" ]; then \
      . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
      . /workspace/install/setup.bash && \
      /venv/bin/python3 -m demnav_pkg.seed_cache \
        --lat "$NAV_ASSET_LAT" \
        --lon "$NAV_ASSET_LON" \
        --area-km "$DEMNAV_AREA_KM" \
        --resolution-m "$DEMNAV_DEM_RESOLUTION_M" \
        --cache-dir /data/demnav_cache; \
    fi

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

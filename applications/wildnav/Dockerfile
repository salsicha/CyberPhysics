ARG TAG=amd64

FROM cyberphysics/aerostack2:${TAG} AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    ROS2_DISTRO=humble \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

ARG SEED_NAV_ASSETS=1
ARG NAV_ASSET_LAT=37.7694
ARG NAV_ASSET_LON=-122.4862
ARG WILDNAV_ZOOM=18
ARG WILDNAV_SEARCH_RADIUS_M=250.0
ARG WILDNAV_MAX_TILES=49
ARG WILDNAV_FEATURE_BACKEND=SIFT
ARG WILDNAV_TILE_URL=https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}

WORKDIR /workspace

RUN apt update && apt install -q -y --no-install-recommends \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*
COPY src /workspace/src

RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    colcon build --merge-install --install-base /workspace/install

RUN if [ "$SEED_NAV_ASSETS" = "1" ]; then \
      . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
      . /workspace/install/setup.bash && \
      /venv/bin/python3 -m wildnav_pkg.seed_cache \
        --lat "$NAV_ASSET_LAT" \
        --lon "$NAV_ASSET_LON" \
        --zoom "$WILDNAV_ZOOM" \
        --search-radius-m "$WILDNAV_SEARCH_RADIUS_M" \
        --max-tiles "$WILDNAV_MAX_TILES" \
        --feature-backend "$WILDNAV_FEATURE_BACKEND" \
        --tile-url-template "$WILDNAV_TILE_URL" \
        --cache-dir /data/wildnav_cache; \
    fi

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

FROM scratch
COPY --from=build / /

ENTRYPOINT ["/entrypoint.sh"]

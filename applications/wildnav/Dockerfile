ARG TAG=latest

FROM salsicha/cyberphysics:aerostack2-${TAG}

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

# colcon writes its own interpreter into the console scripts' shebangs: run
# it with the venv's, so the nodes see the packages installed there
# (synthetic_world), not the bare /usr/bin/python3.
RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    /venv/bin/python3 /usr/bin/colcon build --merge-install --install-base /workspace/install
# Each console script must import its module with its own interpreter
# (the plain ROS environment, as ros2 launch runs it).
RUN . /opt/ros/$ROS2_DISTRO/setup.bash && . /workspace/install/setup.bash && \
    cd / && for script in /workspace/install/lib/wildnav_pkg/*; do \
      "$(sed -n '1s/^#!//p' "$script")" -c 'import sys; from importlib.metadata import entry_points; [ep] = [e for e in entry_points(group="console_scripts") if e.name == sys.argv[1]]; ep.load()' "${script##*/}" || exit 1; \
    done

# The seed goes beside the cache, not in it: the vehicle stacks mount a
# host directory over /data/wildnav_cache (compositions/blueos_nav_core.yaml)
# and would hide it. The entrypoint copies the seed files the cache lacks
# into it on every start (applications/wildnav/README.md).
ENV WILDNAV_SEED_DIR=/usr/share/cyberphysics/wildnav_seed
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
        --cache-dir "$WILDNAV_SEED_DIR"; \
    fi

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Containers start in / (the build steps above ran in /workspace)
WORKDIR /

ENTRYPOINT ["/entrypoint.sh"]

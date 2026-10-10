ARG TAG=latest

FROM salsicha/cyberphysics:aerostack2-${TAG}

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

# colcon writes its own interpreter into the console scripts' shebangs: run
# it with the venv's, so the nodes see the packages installed there
# (synthetic_world, srtm.py), not the bare /usr/bin/python3.
RUN . /venv/bin/activate && . /opt/ros/$ROS2_DISTRO/setup.bash && \
    /venv/bin/python3 /usr/bin/colcon build --merge-install --install-base /workspace/install
# Each console script must import its module with its own interpreter
# (the plain ROS environment, as ros2 launch runs it).
RUN . /opt/ros/$ROS2_DISTRO/setup.bash && . /workspace/install/setup.bash && \
    cd / && for script in /workspace/install/lib/demnav_pkg/*; do \
      "$(sed -n '1s/^#!//p' "$script")" -c 'import sys; from importlib.metadata import entry_points; [ep] = [e for e in entry_points(group="console_scripts") if e.name == sys.argv[1]]; ep.load()' "${script##*/}" || exit 1; \
    done

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

# Containers start in / (the build steps above ran in /workspace)
WORKDIR /

ENTRYPOINT [ "/entrypoint.sh" ]

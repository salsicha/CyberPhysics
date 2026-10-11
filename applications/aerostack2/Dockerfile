ARG TAG=latest

FROM ros:humble-ros-base

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=humble

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# The main ROS 2 Humble repository dropped mavros, mavros-extras and
# libmavconn in its September 2026 sync (only mavros-msgs 2.15.1 synced).
# Install the MAVROS 2.14.0 set this stack was validated with from the
# dated ROS snapshot; everything else still comes from the main repository.
# Aerostack2's Gazebo assets are pinned to the Aerostack2 release of the
# other as2 packages (1.1.3): compositions/aerostack2_sim.yaml runs their
# launch_simulation.py and bind-mounts its own drone_bridges.py and
# quadrotor_base, hd_camera and rgbd_camera templates over this layout
# (asserted below).
# The pins are upstream versions, applied by apt preferences: the build farm
# builds each architecture separately, so a full Debian version
# (2.14.0-1jammy.<build date>) exists for only one of amd64 and arm64, and
# the snapshot has no arm64 as2-gazebo-assets (the main repository does).
ARG ROS_SNAPSHOT=2026-08-07
ARG MAVROS_VERSION=2.14.0
ARG MAVLINK_VERSION=2026.6.6
ARG AS2_GAZEBO_ASSETS_VERSION=1.1.3
COPY ros-snapshots.asc /tmp/ros-snapshots.asc
RUN gpg --show-keys --with-colons /tmp/ros-snapshots.asc \
      | grep -q '^fpr:::::::::4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA:' && \
    gpg --dearmor -o /usr/share/keyrings/ros-snapshots.gpg /tmp/ros-snapshots.asc && \
    echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.gpg] http://snapshots.ros.org/humble/${ROS_SNAPSHOT}/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros-snapshots.list && \
    rm /tmp/ros-snapshots.asc && \
    printf '%s\n' \
      "Package: ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-libmavconn" \
      "Pin: version ${MAVROS_VERSION}-1jammy.*" \
      "Pin-Priority: 1001" \
      "" \
      "Package: ros-${ROS_DISTRO}-mavlink" \
      "Pin: version ${MAVLINK_VERSION}-1jammy.*" \
      "Pin-Priority: 1001" \
      "" \
      "Package: ros-${ROS_DISTRO}-as2-gazebo-assets" \
      "Pin: version ${AS2_GAZEBO_ASSETS_VERSION}-1jammy.*" \
      "Pin-Priority: 1001" \
      > /etc/apt/preferences.d/cyberphysics-pins

RUN apt update && apt install -q -y --no-install-recommends \
    python3-pip python3-venv python3-setuptools libgl1-mesa-dri \
    ros-${ROS_DISTRO}-as2-core ros-${ROS_DISTRO}-as2-msgs \
    ros-${ROS_DISTRO}-as2-state-estimator \
    ros-${ROS_DISTRO}-as2-motion-controller \
    ros-${ROS_DISTRO}-as2-behaviors-motion \
    ros-${ROS_DISTRO}-as2-behaviors-trajectory-generation \
    ros-${ROS_DISTRO}-as2-behaviors-perception \
    ros-${ROS_DISTRO}-as2-behavior-tree \
    ros-${ROS_DISTRO}-as2-platform-gazebo \
    ros-${ROS_DISTRO}-as2-gazebo-assets \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-libmavconn \
    ros-${ROS_DISTRO}-mavlink \
    ros-${ROS_DISTRO}-rosbridge-suite \
    geographiclib-tools python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    for pin in mavros:${MAVROS_VERSION} mavros-extras:${MAVROS_VERSION} \
        mavros-msgs:${MAVROS_VERSION} libmavconn:${MAVROS_VERSION} \
        mavlink:${MAVLINK_VERSION} as2-gazebo-assets:${AS2_GAZEBO_ASSETS_VERSION}; do \
      installed=$(dpkg-query -W -f '${Version}' "ros-${ROS_DISTRO}-${pin%%:*}") && \
      case "${installed}" in "${pin#*:}-1jammy."*) ;; \
        *) echo "ros-${ROS_DISTRO}-${pin%%:*} is ${installed}, not ${pin#*:}"; exit 1 ;; esac; \
    done

# The Gazebo stack's launch files resolve and every bind-mount target exists.
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    share="$(ros2 pkg prefix as2_gazebo_assets)/share/as2_gazebo_assets" && \
    for file in launch/launch_simulation.py launch/drone_bridges.py \
        models/quadrotor_base/quadrotor_base.sdf.jinja \
        models/hd_camera/hd_camera.sdf.jinja \
        models/rgbd_camera/rgbd_camera.sdf.jinja; do \
      test -f "${share}/${file}" || { echo "missing ${share}/${file}"; exit 1; }; \
    done && \
    ros2 launch as2_gazebo_assets launch_simulation.py --show-args > /dev/null && \
    ros2 launch as2_gazebo_assets drone_bridges.py --show-args > /dev/null

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY requirements.txt /requirements.txt
RUN python3 -m venv --system-site-packages /venv
# Ubuntu 22.04's pip lets the build-isolation environment see the system's
# setuptools 59 (through --system-site-packages), which ignores pyproject
# [project] metadata and installs local packages as "UNKNOWN" without their
# modules or console scripts. A current pip isolates the build properly; the
# venv keeps the system setuptools that colcon needs.
RUN . /venv/bin/activate && pip3 install --upgrade pip==25.2
RUN . /venv/bin/activate && pip3 install -r /requirements.txt

# Shared libraries, importable in this image and every image derived from
# it (demnav, wildnav, boat, submarine): the synthetic-world model and the
# generic MAVROS mirror bridge.
COPY synthetic_world /workspace/synthetic_world
COPY mavros_mirror /workspace/mavros_mirror
RUN . /venv/bin/activate && \
    pip3 install /workspace/synthetic_world && \
    pip3 install /workspace/mavros_mirror && \
    cd / && python3 -c "from synthetic_world import METERS_PER_DEG_LAT" && \
    test -x /venv/bin/mavros_mirror

COPY src /workspace/src
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /workspace && \
    colcon build --merge-install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

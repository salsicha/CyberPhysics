ARG TAG=latest

FROM ros:jazzy-ros-base

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

# Aerostack2 has no Jazzy binary release yet. Pin the upstream source after
# its Jazzy and BehaviorTree.CPP 4 compatibility fixes, before later controller
# and state-estimator refactors. Never build a moving branch in deployment.
ARG AS2_REVISION=97ade9f1d43ac760da8f69312e8ef9d2b4bf2328
RUN apt-get update -o Acquire::Retries=5 && apt-get install -q -y --no-install-recommends \
    git python3-pip python3-venv python3-setuptools python3-rosdep \
    python3-colcon-common-extensions libgl1-mesa-dri libgeographiclib-dev \
    ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    geographiclib-tools

RUN git init /as2_ws/src/aerostack2 && \
    git -C /as2_ws/src/aerostack2 remote add origin https://github.com/aerostack2/aerostack2.git && \
    (for attempt in 1 2 3; do \
      git -c http.version=HTTP/1.1 -C /as2_ws/src/aerostack2 \
        fetch --depth 1 origin "$AS2_REVISION" && exit 0; \
      sleep 5; \
    done; exit 1) && \
    git -C /as2_ws/src/aerostack2 checkout --detach FETCH_HEAD && \
    test "$(git -C /as2_ws/src/aerostack2 rev-parse HEAD)" = "$AS2_REVISION"

ARG AS2_PACKAGES="as2_state_estimator as2_motion_controller as2_behaviors_motion as2_behaviors_trajectory_generation as2_behavior_tree as2_platform_gazebo as2_gazebo_assets"
ARG BUILD_JOBS=3
# colcon supplies Make's own -j flag unless MAKEFLAGS already limits it;
# CMAKE_BUILD_PARALLEL_LEVEL alone does not bound a Unix Makefiles build.
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && cd /as2_ws && \
    { test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init; } && \
    rosdep update --rosdistro "$ROS_DISTRO" && \
    rosdep install --from-paths $(colcon list --paths-only --packages-up-to $AS2_PACKAGES) \
      --ignore-src --rosdistro "$ROS_DISTRO" -t build -t buildtool -t build_export -t exec -y && \
    MAKEFLAGS="-j${BUILD_JOBS} -l${BUILD_JOBS}" CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" \
      colcon build --merge-install --executor sequential \
      --install-base /opt/aerostack2 --packages-up-to $AS2_PACKAGES \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 && \
    rm -rf /as2_ws /var/lib/apt/lists/*

# Assert the source-built layout used by the Gazebo composition's mounts.
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && . /opt/aerostack2/setup.bash && \
    share="$(ros2 pkg prefix as2_gazebo_assets)/share/as2_gazebo_assets" && \
    for file in launch/launch_simulation.py launch/drone_bridges.py \
        models/quadrotor_base/quadrotor_base.sdf.jinja \
        models/hd_camera/hd_camera.sdf.jinja \
        models/rgbd_camera/rgbd_camera.sdf.jinja; do \
      test -f "${share}/${file}" || { echo "missing ${share}/${file}"; exit 1; }; \
    done && \
    ros2 launch as2_gazebo_assets launch_simulation.py --show-args > /dev/null && \
    ros2 launch as2_gazebo_assets drone_bridges.py --show-args > /dev/null && \
    python3 -c "from vision_msgs.msg import Detection2DArray"

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY requirements.txt /requirements.txt
RUN python3 -m venv --system-site-packages /venv
# Pin the Python build frontend while retaining the system ROS/colcon modules
# in the venv. Local pyproject packages must keep their module/entry-point metadata.
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
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && . /opt/aerostack2/setup.bash && \
    cd /workspace && \
    MAKEFLAGS="-j${BUILD_JOBS} -l${BUILD_JOBS}" CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" \
      colcon build --merge-install && \
    . /workspace/install/setup.bash && \
    ros2 launch as2_platform_blueos blueos_autonomy.launch.py --show-args > /dev/null

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ZENOH_ROUTER_CHECK_ATTEMPTS=-1 \
    ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true;listen/endpoints=["tcp/0.0.0.0:0"];connect/endpoints=[]'

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

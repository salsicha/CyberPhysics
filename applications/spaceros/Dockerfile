
# Pinned Space ROS release; the demos checkout below uses the matching tag.
ARG SPACEROS_RELEASE=jazzy-2026.07.0
FROM osrf/space-ros:${SPACEROS_RELEASE}
ARG SPACEROS_RELEASE

# The base image runs as spaceros-user; installing packages and creating /venv
# need root, and the container runs as root
USER root

# This pinned Space ROS core links directly to Cyclone DDS (runtime RMW
# selection is absent). Adding an apt RMW cannot switch it; keep this graph
# on Cyclone until the Space ROS core itself is rebuilt and validated.
ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH" \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GZ_VERSION=harmonic

# This prevents ROS setup.bash from failing
SHELL ["/bin/bash","-c"]

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl gnupg2 lsb-release python3-pip python3-venv vim wget && \
    rm -rf /var/lib/apt/lists/*

# System site packages keep rclpy's apt dependencies (yaml, numpy, ...)
# importable from /venv's python3, which is first on PATH
RUN python3 -m venv --system-site-packages /venv

# Space ROS demos at the tag matching the base image. Only curiosity_rover is
# checked out; its build.sh/run.sh wrap docker compose, so the steps of its
# Dockerfile (Space ROS side) and Dockerfile.GUI (Gazebo side) are reproduced
# below instead.
RUN git clone --depth 1 --filter=blob:none --sparse --branch "${SPACEROS_RELEASE}" \
        https://github.com/space-ros/demos.git /demos && \
    git -C /demos sparse-checkout set curiosity_rover

# Dockerfile: ROS 2 packages Space ROS does not ship (space-ros/space-ros#195).
# Dockerfile.GUI: Gazebo Harmonic through ros_gz (the Jazzy gz_*_vendor
# packages, instead of a second copy from packages.osrfoundation.org), the
# bridges, xacro (imported by curiosity_gazebo.launch.py but missing from its
# package.xml) and RViz. Mesa provides software GL/EGL for the camera and
# gpu_lidar sensors when no GPU is passed in.
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-sdformat-urdf \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-actuator-msgs \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    libgl1-mesa-dri libegl1 libegl-mesa0 && \
    rm -rf /var/lib/apt/lists/*

# Remaining dependencies of the three demo packages (ros2_control,
# gz_ros2_control, ros2_controllers, ...) via rosdep. With the Space ROS
# underlay sourced, --ignore-src skips the keys Space ROS already provides.
WORKDIR /demos/curiosity_rover
RUN rosdep init && rosdep update --rosdistro "${ROS_DISTRO}" && \
    apt-get update && \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source "${SPACEROS_DIR}/setup.bash" && \
    rosdep install --from-paths . --ignore-src --rosdistro "${ROS_DISTRO}" -y && \
    rm -rf /var/lib/apt/lists/*

# Build curiosity_description, curiosity_gazebo and curiosity_rover_demo in
# one workspace on top of ROS 2 Jazzy + Space ROS (the upstream Dockerfile's
# underlay order)
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source "${SPACEROS_DIR}/setup.bash" && \
    colcon build --event-handlers console_direct- \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log

COPY launch/ /launch/

COPY scripts/ /scripts/
RUN chmod +x /scripts/*.py

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

# `ros2 launch` as PID 1 ignores SIGTERM; rclpy nodes stop cleanly on SIGINT
STOPSIGNAL SIGINT

# Gazebo (with GUI) + ros2_control + the Space ROS demo nodes; pass gui:=false
# for a headless server
CMD [ "ros2", "launch", "/launch/curiosity_sim.launch.py" ]

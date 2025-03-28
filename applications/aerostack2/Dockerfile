
# FROM cyberphysics/ros2:latest AS build
FROM osrf/ros:humble as build

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
    curl gnupg2 lsb-release vim wget ros-{ROS_DISTRO}-aerostack2 && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv
RUN . /venv/bin/activate && pip3 install rpi-hardware-pwm smbus smbus2

COPY scripts/ /scripts/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]

# Launch aerostack2 simulated drone platform (gazebo needs to be running)
CMD ["ros2", "launch", "as2_platform_gazebo", "platform_gazebo_launch.py", "namespace:=drone_sim_0", "simulation_config_file:=world_json_path"]


# Keyboard teleoperation:
# ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py namespace:=<%= drone_namespace %>

# Platform
# ros2 launch as2_platform_multirotor_simulator as2_platform_multirotor_simulator.launch.py namespace:=<%= drone_namespace %> config_file:=config/config.yaml uav_config:=config/uav_config.yaml world_config:=<%= simulation_config_file %>

# State Estimation:
# ros2 launch as2_state_estimator state_estimator_launch.py namespace:=<%= drone_namespace %> config_file:=config/config.yaml

# Motion Control:
# ros2 launch as2_motion_controller controller_launch.py namespace:=<%= drone_namespace %> config_file:=config/config.yaml plugin_name:=<%= motion_controller_plugin %> plugin_config_file:=config/<%= motion_controller_plugin %>.yaml

# Motion behaviors
# ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=<%= drone_namespace %> config_file:=config/config.yaml

# Trajectory Generation Behavior
# ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py namespace:=<%= drone_namespace %> config_file:=config/config.yaml

# Point Gimbal Behavior
# ros2 launch as2_behaviors_perception point_gimbal_behavior.launch.py namespace:=<%= drone_namespace %> config_file:=config/config.yaml

# Behavior tree
# ros2 launch as2_behavior_tree behavior_trees.launch.py drone_id:=<%= drone_namespace %> tree:=./trees/square.xml

# Mission monitoring
# ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node --ros-args -r  __ns:=/<%= drone_namespace %>

#!/bin/bash
# Restore runtime RMW selection and generate the message support Zenoh uses.
# Keep the exact ROS source revisions recorded by the pinned Space ROS image.
set -eo pipefail
python3 - <<'PY'
import pathlib
import re
import subprocess
import yaml

repos = yaml.safe_load(pathlib.Path('/opt/ros/spaceros/scripts/spaceros.repos').read_text())['repositories']
root = pathlib.Path('/tmp/spaceros_zenoh/src')
root.mkdir(parents=True)
for name in ('rmw_implementation', 'rcl', 'rclcpp', 'rclpy', 'rcl_interfaces',
             'common_interfaces', 'unique_identifier_msgs', 'geometry2',
             'rmw_dds_common', 'ros2cli', 'test_interface_files'):
    entry = repos[name]
    if not re.fullmatch(r'[0-9a-f]{40}', entry['version']):
        raise RuntimeError(f'{name} must have an immutable source revision')
    target = root / name
    subprocess.run(['git', 'init', '-q', str(target)], check=True)
    subprocess.run(['git', '-C', str(target), 'fetch', '-q', '--depth', '1',
                    entry['url'], entry['version']], check=True)
    subprocess.run(['git', '-C', str(target), 'checkout', '-q', '--detach', 'FETCH_HEAD'], check=True)
PY
# ROS setup scripts do not support nounset.
source /opt/ros/jazzy/setup.bash
source /opt/ros/spaceros/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
unset STATIC_ROSIDL_TYPESUPPORT_C STATIC_ROSIDL_TYPESUPPORT_CPP
cd /tmp/spaceros_zenoh
mapfile -t interfaces < <(find /opt/ros/spaceros/share/ament_index/resource_index/rosidl_interfaces -maxdepth 1 -type f -printf '%f\n')
# Old message libraries return only introspection support; Zenoh needs the
# Fast CDR serialization support as well. Rebuild every installed interface,
# then the RMW dispatcher and C/C++/Python client libraries in dependency order.
MAKEFLAGS='-j2 -l2' colcon build --merge-install --executor sequential \
    --install-base /opt/ros/spaceros \
    --packages-select "${interfaces[@]}" rmw_implementation rcl rcl_action rcl_lifecycle \
      rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle rclpy \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
      -DRMW_IMPLEMENTATION=rmw_zenoh_cpp \
      -DRMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION=OFF \
      -DSTATIC_ROSIDL_TYPESUPPORT_C= -DSTATIC_ROSIDL_TYPESUPPORT_CPP= \
      -DPython3_EXECUTABLE=/usr/bin/python3
cd /
rm -rf /tmp/spaceros_zenoh

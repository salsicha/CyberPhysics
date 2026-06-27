#!/usr/bin/env bash
set -euo pipefail

OUTPUT=${1:-/tmp/so101_groot_demo_bag}
ros2 bag record \
  --output "${OUTPUT}" \
  /joint_states \
  /so101/joint_commands \
  /so101/camera/image_raw \
  /so101/camera/camera_info \
  /so101/camera/depth/image_rect_raw \
  /so101/camera/depth/camera_info \
  /so101/camera/imu \
  /tf \
  /tf_static

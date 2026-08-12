#!/bin/bash
set -e

if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi

if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

if [ -f /venv/bin/activate ]; then
    source /venv/bin/activate
fi

# Both the physical and simulated turret mount these locations from the host.
# Create them here so Ultralytics and Torch never fall back to a read-only home.
if [ -n "${YOLO_CONFIG_DIR:-}" ]; then
    mkdir -p "${YOLO_CONFIG_DIR}"
fi
if [ -n "${TORCH_HOME:-}" ]; then
    mkdir -p "${TORCH_HOME}"
fi
if [ -n "${PANTILT_MODEL_DIR:-}" ]; then
    mkdir -p "${PANTILT_MODEL_DIR}"
fi

exec "$@"

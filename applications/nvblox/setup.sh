#!/bin/bash
set -euo pipefail

# isaac_ros_nvblox_assets is pinned to the release used with Isaac ROS 4.x
# (quickstart, galileo_static_3_2, galileo_people_3_2). This used to pick the
# newest 4.x version through NGC's catalog listing API, which now returns 404;
# 5.0.0 targets Isaac ROS 5.
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_nvblox_assets"
NGC_FILENAME="quickstart.tar.gz"
NGC_VERSION="${ISAAC_ROS_NVBLOX_ASSETS_VERSION:-4.0.0}"

mkdir -p /workspace/isaac_ros_assets
FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/${NGC_ORG}/${NGC_TEAM}/${NGC_RESOURCE}/versions/${NGC_VERSION}/files/${NGC_FILENAME}"
curl -fL --request GET "${FILE_REQ_URL}" -o "${NGC_FILENAME}"
tar -xf "${NGC_FILENAME}" -C /workspace/isaac_ros_assets
rm "${NGC_FILENAME}"

#!/bin/bash
set -euo pipefail

NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_nvblox_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION="${ISAAC_ROS_RELEASE_MAJOR:-4}"
MINOR_VERSION="${ISAAC_ROS_RELEASE_MINOR:-4}"
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=${NGC_ORG}&teamName=${NGC_TEAM}&name=${NGC_RESOURCE}&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"

AVAILABLE_VERSIONS=$(curl -fsSL -H "Accept: application/json" "${VERSION_REQ_URL}")
LATEST_VERSION_ID=$(echo "${AVAILABLE_VERSIONS}" | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == ${MAJOR_VERSION} and .minor <= ${MINOR_VERSION})
    | \$v
    " | sort -V | tail -n 1)

if [ -z "${LATEST_VERSION_ID}" ]; then
    echo "No corresponding version found for Isaac ROS ${MAJOR_VERSION}.${MINOR_VERSION}" >&2
    echo "Found versions:" >&2
    echo "${AVAILABLE_VERSIONS}" | jq -r '.recipeVersions[].versionId' >&2
    exit 1
fi

mkdir -p /workspace/isaac_ros_assets
FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/${NGC_ORG}/${NGC_TEAM}/${NGC_RESOURCE}/versions/${LATEST_VERSION_ID}/files/${NGC_FILENAME}"
curl -fL --request GET "${FILE_REQ_URL}" -o "${NGC_FILENAME}"
tar -xf "${NGC_FILENAME}" -C /workspace/isaac_ros_assets
rm "${NGC_FILENAME}"

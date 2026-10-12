#!/bin/bash
# Exercise the built images without connecting to a robot or starting actuators.
# Run on the Jetson after building: tools/jetson_gpu_smoke.sh [all|APP]
set -euo pipefail

app=${1:-all}
case "$app" in
    all) apps=(nvblox pantiltros depthanything) ;;
    nvblox|pantiltros|depthanything) apps=("$app") ;;
    *) echo "Usage: $0 [all|nvblox|pantiltros|depthanything]" >&2; exit 2 ;;
esac

if [ "$(uname -m)" != aarch64 ]; then
    echo 'Run this check on the deployment Jetson (aarch64).' >&2
    exit 1
fi
if ! grep -Eq 'R39 .*REVISION: 2\.' /etc/nv_tegra_release; then
    echo 'These arm64 GPU images require JetPack 7.2 / Jetson Linux 39.2.x.' >&2
    exit 1
fi

repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
python3 "$repo_dir/tools/configure_dds_buffers.py" --check
for app in "${apps[@]}"; do
    image="${REPOSITORY:-salsicha/cyberphysics}:${app}-arm64"
    [ "$(docker image inspect -f '{{.Architecture}}' "$image")" = arm64 ] || {
        echo "${image} is not an arm64 image" >&2; exit 1;
    }
    echo "Checking ${image} on the Jetson GPU"
    docker run --rm --network none --runtime nvidia --gpus all \
        -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e ROS_DOMAIN_ID=219 -e ROS_LOCALHOST_ONLY=1 \
        -v "$repo_dir/tools/jetson_gpu_smoke.py:/tmp/jetson_gpu_smoke.py:ro" \
        "$image" python3 /tmp/jetson_gpu_smoke.py "$app"
done

#!/bin/bash
# Point the runner's Docker daemon at Google's public Docker Hub mirror, so
# base image pulls do not hit Docker Hub's anonymous pull rate limit (429 Too
# Many Requests). No credentials are involved, and Docker falls back to Docker
# Hub for images the mirror does not have. Other settings already in
# /etc/docker/daemon.json are kept.
set -euo pipefail

mirror=https://mirror.gcr.io
cfg=${DOCKER_DAEMON_JSON:-/etc/docker/daemon.json}

current='{}'
if sudo test -s "$cfg"; then
    current=$(sudo cat "$cfg")
fi
updated=$(jq --arg m "$mirror" \
    '."registry-mirrors" = ([$m] + ((."registry-mirrors" // []) - [$m]))' <<< "$current")
sudo mkdir -p "$(dirname "$cfg")"
sudo tee "$cfg" <<< "$updated" > /dev/null
cat "$cfg"

sudo systemctl restart docker
docker info | grep -A1 'Registry Mirrors'

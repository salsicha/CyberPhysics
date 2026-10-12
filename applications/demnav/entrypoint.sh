#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/aerostack2/setup.bash
source /venv/bin/activate
source /workspace/install/setup.bash

# Copy the image's build-time seed (Dockerfile, DEMNAV_SEED_DIR)
# into the cache: only the files the cache does not have, and not when
# it is mounted read-only. The vehicle stacks mount a host directory
# over the cache, which hid the seed (compositions/blueos_nav_core.yaml,
# applications/demnav/README.md).
seed_cache() {
    local seed=$1 cache=$2 copied
    [ -d "$seed" ] && [ -n "$(ls -A "$seed")" ] || return 0
    mkdir -p "$cache" 2> /dev/null && [ -w "$cache" ] || return 0
    copied=$(cp -Rnv "$seed/." "$cache/" 2> /dev/null | wc -l)
    if [ "$copied" -gt 0 ]; then
        echo "Seeded $cache with $copied file(s) from the image's $seed" >&2
    fi
}
seed_cache "${DEMNAV_SEED_DIR:-/usr/share/cyberphysics/demnav_seed}" /data/demnav_cache
exec "$@"

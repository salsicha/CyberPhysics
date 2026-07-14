#!/bin/bash

# automatically configure the Blackfly's IP based on current interface subnet
# ./blackfly-autoconfigure-ip.sh [expected number of cameras]

EXPECTED_NUM_BLACKFLY_CAMERAS=${1:-1}
MAX_ATTEMPTS=${2:-60}

attempt=0
while true
do
    cameras_found=`/opt/spinnaker/bin/GigEConfig -a | grep 'serial number' | wc -l`

    echo "Cameras found: $cameras_found/$EXPECTED_NUM_BLACKFLY_CAMERAS"

    if [[ $cameras_found -ge $EXPECTED_NUM_BLACKFLY_CAMERAS ]]; then
        exit 0
    fi

    attempt=$((attempt + 1))
    if [[ $attempt -ge $MAX_ATTEMPTS ]]; then
        echo "Timed out waiting for $EXPECTED_NUM_BLACKFLY_CAMERAS camera(s) after $MAX_ATTEMPTS attempts" >&2
        exit 1
    fi

    sleep 1
done

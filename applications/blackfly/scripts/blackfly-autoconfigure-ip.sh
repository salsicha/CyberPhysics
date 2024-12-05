#!/bin/bash

# automatically configure the Blackfly's IP based on current interface subnet
# ./blackfly-autoconfigure-ip.sh [expected number of cameras]

EXPECTED_NUM_BLACKFLY_CAMERAS=${1:-1}

while true
do
    cameras_found=`/opt/spinnaker/bin/GigEConfig -a | grep 'serial number' | wc -l`

    echo "Cameras found: $cameras_found/$EXPECTED_NUM_BLACKFLY_CAMERAS"

    if [[ $cameras_found -ge $EXPECTED_NUM_BLACKFLY_CAMERAS ]]; then
        exit 0
    fi
done

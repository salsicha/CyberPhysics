#!/bin/bash
set -e

if [ $# -lt 1 ]; then
    echo "./start.sh [system]"
    exit 1
fi

# The argument to this script must match the system name in the "systems" folder
env_file="./systems/$1/"
if [ ! -d "$env_file" ]; then
    echo "System $env_file does not exist"
    exit 1
fi

set -a
source ./systems/$1/env/env.list
set +a

# Run the startup script if it exists
# Also pass any arguments the user supplied
startup_script="./systems/$1/scripts/startup.sh"
if [ -f "$startup_script" ]; then
    echo "Running $startup_script"
    source $startup_script $@
fi

# This can move to ./systems/$1/scripts/startup.sh
if [ "$OFFSCREEN" == "true" ]; then
    ./apps/airsim_server/server/Development/Linux/GeoSpecificEnv.sh -RenderOffScreen &
fi

cd deployment/components/

OUT=""
for A in $COMPONENT_LIST; do
    OUT=$OUT" -f "$A".yml";
done;

docker compose $OUT up --remove-orphans

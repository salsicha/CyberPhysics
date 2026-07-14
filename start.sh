#!/bin/bash
set -e

if [ $# -lt 1 ]; then
    echo "./start.sh [system]"
    exit 1
fi

# The argument to this script must match the system name in the "systems" folder
system_dir="./systems/$1"
if [ ! -d "$system_dir" ]; then
    echo "System $system_dir does not exist"
    exit 1
fi

# Load the system's environment if it has one
env_file="$system_dir/env.list"
if [ -f "$env_file" ]; then
    set -a
    source "$env_file"
    set +a
fi

# Run the startup script if it exists
# Also pass any arguments the user supplied
startup_script="$system_dir/scripts/startup.sh"
if [ -f "$startup_script" ]; then
    echo "Running $startup_script"
    source "$startup_script" "$@"
fi

if [ -z "$COMPONENT_LIST" ]; then
    echo "COMPONENT_LIST is not set (define it in $env_file or $startup_script)"
    exit 1
fi

cd systems/components/

compose_files=()
for A in $COMPONENT_LIST; do
    compose_files+=(-f "$A.yaml")
done

docker compose "${compose_files[@]}" up --remove-orphans

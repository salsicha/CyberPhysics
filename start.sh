#!/bin/bash
# Start a compose stack from compositions/.
#
#   ./start.sh <stack> [--profile <name>]... [-f <override.yaml>]... [docker compose up args...]
#
# is shorthand for
#
#   docker compose -f compositions/<stack>.yaml [-f <override.yaml>]... [--profile <name>]... up [args...]
#
# Examples:
#   ./start.sh racecarneo
#   ./start.sh turret_sim --profile validation
#   ./start.sh blackfly -f systems/components/record-host-shutdown.yaml
#   ./start.sh vinsmono --build
#   ./start.sh marimo -d    # detached; stop it with
#                           # docker compose -f compositions/marimo.yaml down --remove-orphans
set -e

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
compositions_dir="${repo_dir}/compositions"

list_stacks() {
    echo "Available stacks:"
    for f in "${compositions_dir}"/*.yaml; do
        name="$(basename "$f" .yaml)"
        echo "    ${name}"
    done
}

if [ $# -lt 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "Usage: ./start.sh <stack> [--profile <name>]... [-f <override.yaml>]... [docker compose up args...]"
    echo
    list_stacks
    exit 1
fi

# Accept "racecarneo", "racecarneo.yaml" or "compositions/racecarneo.yaml"
stack="$(basename "$1" .yaml)"
shift
compose_file="${compositions_dir}/${stack}.yaml"
if [ ! -f "$compose_file" ]; then
    echo "Unknown stack '${stack}': ${compose_file} does not exist"
    echo
    list_stacks
    exit 1
fi

# --profile and extra compose files (-f) are global docker compose options,
# so they go before "up"
global_args=()
up_args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --profile|-f|--file)
            if [ $# -lt 2 ]; then
                echo "$1 needs a value"
                exit 1
            fi
            global_args+=("$1" "$2")
            shift 2
            ;;
        --profile=*|--file=*)
            global_args+=("$1")
            shift
            ;;
        *)
            up_args+=("$1")
            shift
            ;;
    esac
done

set -x
exec docker compose -f "$compose_file" "${global_args[@]}" up "${up_args[@]}"

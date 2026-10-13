#!/bin/bash
# Generate a matched Isaac dataset and train the compact PerAct demo locally.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(docker compose -f "$repo_dir/compositions/franka_peract_isaac.yaml")
export PERACT_HEADLESS=true
"${compose[@]}" --profile sim stop isaacsim peract_policy
"${compose[@]}" --profile collect run --rm collect
"${compose[@]}" --profile train run --rm train
echo "PerAct preparation complete. Run: PERACT_HEADLESS=true ./start.sh franka_peract_isaac --profile sim"

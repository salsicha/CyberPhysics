#!/bin/bash
# Generate a matched Isaac dataset and train the Act3D demo locally.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose=(docker compose -f "$repo_dir/compositions/franka_act3d_isaac.yaml")
export ACT3D_HEADLESS=true
"${compose[@]}" --profile sim stop isaacsim act3d_policy
"${compose[@]}" --profile collect run --rm collect
"${compose[@]}" --profile train run --rm train
echo "Act3D preparation complete. Run: ACT3D_HEADLESS=true ./start.sh franka_act3d_isaac --profile sim"

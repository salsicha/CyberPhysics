#!/bin/bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
xacro "$root/urdf/so101.urdf.xacro" use_gazebo:=false -o "$root/urdf/so101.urdf"

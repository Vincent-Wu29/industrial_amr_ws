#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_workspace="${AMR_BUILD_WS:-$HOME/industrial_amr_ws}"
if [[ "$build_workspace" == / || "$build_workspace" == "$HOME" || -z "$build_workspace" ]]; then
  echo 'Refusing unsafe build workspace' >&2; exit 2
fi
if [[ -d "$build_workspace" && ! -f "$build_workspace/.industrial-amr-owned" ]]; then
  echo "Existing unowned workspace: $build_workspace; set AMR_BUILD_WS to a new directory" >&2; exit 2
fi
mkdir -p "$build_workspace/src"
touch "$build_workspace/.industrial-amr-owned"
# Linux files avoid rosidl path parsing failures on Windows paths with spaces.
cp -a "$project_dir/src/." "$build_workspace/src/"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$build_workspace"
colcon build --base-paths src --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
echo "Built: $build_workspace. Source $build_workspace/install/setup.bash"

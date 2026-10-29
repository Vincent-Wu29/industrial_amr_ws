#!/usr/bin/env bash
# Source from each shell so clients, Gazebo plugins and Nav2 share one RMW.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Source this file: source tools/runtime_env.sh' >&2
  exit 2
fi
amr_runtime_workspace="${AMR_BUILD_WS:-$HOME/industrial_amr_ws}"
if [[ ! -f "$amr_runtime_workspace/.industrial-amr-owned" || ! -f "$amr_runtime_workspace/install/setup.bash" ]]; then
  echo 'Build the owned AMR workspace with tools/wsl_build.sh first' >&2
  return 1
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source /opt/ros/jazzy/setup.bash
source "$amr_runtime_workspace/install/setup.bash"
printf 'AMR runtime: %s, workspace: %s\n' "$RMW_IMPLEMENTATION" "$amr_runtime_workspace"

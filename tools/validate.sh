#!/usr/bin/env bash
# Full, reproducible validation entrypoint. Long physical suites are opt-in.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
stage="${1:-unit}"
build_workspace="${AMR_BUILD_WS:-$HOME/industrial_amr_ws}"
factor="${AMR_REAL_TIME_FACTOR:-1.0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
bash "$project_dir/tools/wsl_build.sh"
source "$build_workspace/install/setup.bash"
cd "$build_workspace"
run_dir="${AMR_VALIDATION_DIR:-artifacts/acceptance_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$run_dir"
unit() {
  local diagnostics="$build_workspace/artifacts/unit_diagnostics/$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$diagnostics"
  printf '%s\n' "$diagnostics" > "$run_dir/unit_diagnostics_path.txt"
  TMPDIR="$diagnostics" PYTEST_ADDOPTS="${PYTEST_ADDOPTS:-} --showlocals" colcon test --packages-select amr_mission amr_navigation amr_simulation amr_bringup --event-handlers console_direct+ | tee "$run_dir/unit.log"
  colcon test-result --verbose | tee "$run_dir/test_result.log"
}
navigation() { python3 "$project_dir/tools/run_navigation_validation.py" --output "$run_dir/m1" --real-time-factor "$factor"; }
startup() { ros2 run amr_bringup scenario_runner --suite m0_startup --repeat 5 --domain-id 100 --headless --real-time-factor "$factor" --fail-fast --output "$run_dir/startup"; }
m2() { ros2 run amr_bringup scenario_runner --suite m2_happy_path --repeat 10 --seed 42 --headless --real-time-factor "$factor" --fail-fast --output "$run_dir/m2"; }
m3() {
  python3 "$project_dir/tools/fill_normal_samples.py" --output "$run_dir/m3_happy" --seed 42 --count 30 --real-time-factor "$factor"
  faults
}
faults() {
  ros2 run amr_bringup scenario_runner --suite m3_regression --repeat 5 --seed 42 --headless --real-time-factor "$factor" --fail-fast --output "$run_dir/m3_regression"
}
obstacles() {
  # Development regression starts at the observed failing seed; kept outside M3.
  ros2 run amr_bringup scenario_runner --cases crossing temporary --repeat 5 --seed 46 --headless --real-time-factor "$factor" --fail-fast --output "$run_dir/obstacle_probe"
}
freeze_registration() {
  if [[ -f "$run_dir/m4_registration.json" ]]; then
    cmp -s "$project_dir/tools/m4_registration.json" "$run_dir/m4_registration.json" || {
      echo 'Registration changed: choose a new experiment cohort and preserve the old one' >&2; return 1;
    }
  else
    cp "$project_dir/tools/m4_registration.json" "$run_dir/m4_registration.json"
  fi
}
preflight() {
  freeze_registration
  # Functional preflight is retained separately and never enters paired statistics.
  for profile in baseline candidate; do
    ros2 run amr_bringup scenario_runner --suite narrow_aisle --profile "$profile" --repeat 1 --seed 45 --headless --real-time-factor 3 --fail-fast --output "$run_dir/m4_preflight"
  done
}
experiments() {
  freeze_registration
  python3 "$project_dir/tools/check_m4_preflight.py" --input "$run_dir/m4_preflight"
  # Validate the candidate's mission/fault behavior before expensive paired sampling.
  ros2 run amr_bringup scenario_runner --suite m3_regression --profile candidate --repeat 1 --seed 42 --headless --real-time-factor "$factor" --fail-fast --output "$run_dir/m4_regression"
  for profile in baseline candidate; do
    ros2 run amr_bringup scenario_runner --suite narrow_aisle --profile "$profile" --repeat 30 --seed 42 --headless --real-time-factor 3 --fail-fast --output "$run_dir/m4"
  done
  ros2 run amr_bringup metrics_report --input "$run_dir/m4" --compare baseline candidate --metric sim --output "$run_dir/m4_comparison.md"
}
case "$stage" in
  unit) unit;;
  startup) startup;;
  navigation) navigation;;
  m2) m2;;
  m3) m3;;
  faults) faults;;
  obstacles) obstacles;;
  preflight) preflight;;
  experiments) experiments;;
  all) unit; startup; navigation; m2; m3; preflight; experiments;;
  *) echo 'usage: validate.sh unit|startup|navigation|m2|m3|faults|obstacles|preflight|experiments|all' >&2; exit 2;;
esac
ros2 run amr_bringup metrics_report --input "$run_dir" --output "$run_dir/report.md"

#!/usr/bin/env bash
set -euo pipefail
# Requires a running Gazebo, control.launch, and activated velocity plugin.
package_dir="$(rospack find fetch_twist_controller)"
results_dir="${1:-$HOME/IRVL_ws_Official/results/twist_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$results_dir"
recorder_pid=''
cleanup() { if [[ -n "$recorder_pid" ]]; then kill -INT "$recorder_pid" 2>/dev/null || true; wait "$recorder_pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
for frame in base_link gripper_link; do
  python3 "$package_dir/scripts/evaluate.py" --output "$results_dir/$frame.csv" &
  recorder_pid=$!
  sleep 1
  rosrun fetch_twist_controller trajectory.py "$package_dir/config/test_motion.yaml" --frame "$frame"
  sleep 1
  cleanup
  recorder_pid=''
  python3 "$package_dir/scripts/report.py" "$results_dir/$frame.csv"
done
printf 'Results written to %s\n' "$results_dir"

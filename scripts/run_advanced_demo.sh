#!/usr/bin/env bash
set -Eeuo pipefail
ADV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: ./scripts/run_advanced_demo.sh [--preflight|--headless|--startup-check] [--no-rviz] [--hold]'
  echo 'Advanced SLAM and Navigation Demo v0.2: continuous guided mapping, live planning from current position to B.'
  echo 'Ctrl+C requests latched safe stop and cleanup. Recording is a separate process.'
  exit 0
fi
export T1_VENV="${T1_VENV:-/home/haoyang/venvs/humanoid_mujoco}"
export T1_UPSTREAM="${T1_UPSTREAM:-/home/haoyang/robot_sim_sources/booster_deploy}"
export T1_ASSETS="${T1_ASSETS:-/home/haoyang/robot_sim_sources/booster_assets}"
export T1_LOCAL_DDS=1 T1_DDS_PORT=42127 ROS_DOMAIN_ID=37
export T1_DEMO_RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_advanced_$(cat /proc/sys/kernel/random/uuid)"
export T1_DEMO_DIR="$ADV_ROOT/run_data/advanced/$T1_DEMO_RUN_ID"
mkdir -p "$T1_DEMO_DIR"
export ROS_LOG_DIR="$T1_DEMO_DIR/ros-log"
source "$ADV_ROOT/scripts/env.sh" >"$T1_DEMO_DIR/environment.log" 2>&1
[[ -f "$ADV_ROOT/install/setup.bash" ]] || { echo 'Build the independent overlay with system colcon first.' >&2; exit 2; }
"$ADV_ROOT/scripts/demo/build_rviz.sh" >"$T1_DEMO_DIR/rviz-build.log" 2>&1
cmake -S "$ADV_ROOT/scripts/advanced/authority" -B "$ADV_ROOT/build/advanced-tools" -DPython3_EXECUTABLE=/usr/bin/python3 >"$T1_DEMO_DIR/authority-build.log" 2>&1
cmake --build "$ADV_ROOT/build/advanced-tools" --parallel 2 >>"$T1_DEMO_DIR/authority-build.log" 2>&1
runner=''
stop() { if [[ -n "$runner" ]] && kill -0 "$runner" 2>/dev/null; then kill -TERM "$runner"; fi; }
trap stop INT TERM
"$T1_VENV/bin/python" "$ADV_ROOT/scripts/advanced/demo.py" "$@" &
runner=$!
code=0
wait "$runner" || code=$?
if kill -0 "$runner" 2>/dev/null; then wait "$runner" || code=$?; fi
runner=''
if [[ -f "$T1_DEMO_DIR/result.json" ]]; then
  /usr/bin/python3 "$ADV_ROOT/scripts/advanced/finalize.py" "$T1_DEMO_DIR" "$code" || code=$?
fi
exit "$code"

#!/usr/bin/env bash
set -Eeuo pipefail
EXP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: ./scripts/run_exploration_demo.sh [--headless] [--no-rviz] [--hold] [--task FILE] [--static] [--scene NAME_OR_YAML] [--time-limit SECONDS] [--preflight]'
  echo 'W16: MuJoCo + RViz, physical walking, online SLAM, autonomous exploration and real dynamic obstacles.'
  echo 'Default: EXP01 dynamic encounter. --static explicitly removes the obstacle for a control run.'
  echo 'Scene is simulator-only. Default task: src/t1_navigation/config/exploration_task.yaml.'
  exit 0
fi
export T1_VENV="${T1_VENV:-/home/haoyang/venvs/humanoid_mujoco}"
export T1_UPSTREAM="${T1_UPSTREAM:-/home/haoyang/robot_sim_sources/booster_deploy}"
export T1_ASSETS="${T1_ASSETS:-/home/haoyang/robot_sim_sources/booster_assets}"
export T1_LOCAL_DDS=1 T1_DDS_PORT=42127 ROS_DOMAIN_ID=37
export T1_DEMO_RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_exploration_$(cat /proc/sys/kernel/random/uuid)"
export T1_DEMO_DIR="$EXP_ROOT/run_data/exploration/$T1_DEMO_RUN_ID"
mkdir -p "$T1_DEMO_DIR"
export ROS_LOG_DIR="$T1_DEMO_DIR/ros-log"
source "$EXP_ROOT/scripts/env.sh" >"$T1_DEMO_DIR/environment.log" 2>&1
"$EXP_ROOT/scripts/demo/build_rviz.sh" >"$T1_DEMO_DIR/rviz-build.log" 2>&1
cmake -S "$EXP_ROOT/scripts/advanced/authority" -B "$EXP_ROOT/build/advanced-tools" -DPython3_EXECUTABLE=/usr/bin/python3 >"$T1_DEMO_DIR/authority-build.log" 2>&1
cmake --build "$EXP_ROOT/build/advanced-tools" --parallel 2 >>"$T1_DEMO_DIR/authority-build.log" 2>&1
runner=''
stop() { if [[ -n "$runner" ]] && kill -0 "$runner" 2>/dev/null; then kill -TERM "$runner"; fi; }
trap stop INT TERM
"$T1_VENV/bin/python" "$EXP_ROOT/scripts/exploration/demo.py" "$@" &
runner=$!
code=0
wait "$runner" || code=$?
if kill -0 "$runner" 2>/dev/null; then wait "$runner" || code=$?; fi
runner=''
if [[ -f "$T1_DEMO_DIR/result.json" ]]; then
  /usr/bin/python3 "$EXP_ROOT/scripts/advanced/finalize.py" "$T1_DEMO_DIR" "$code" || code=$?
fi
exit "$code"

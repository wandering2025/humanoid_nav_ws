#!/usr/bin/env bash
# One terminal owns one demo; no global installation or process cleanup.
set -Eeuo pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
case "${1:-}" in
  --help|-h)
    cat <<'HELP'
Usage: ./scripts/run_demo.sh [--preflight] [--headless] [--no-rviz] [--scenario U01]
STATUS: W14_BLOCKED after RViz heap corruption on shutdown; GUI locked.
MuJoCo: NVIDIA RTX 4050. RViz: CPU llvmpipe RGB cells (no MapDisplay shader).
Default: MuJoCo + RViz, frozen formal-v1 U01 seed 4000, A(0,0,0) -> B(9,0,0).
--preflight  CPU resource/environment checks only; no graphics/ROS/DDS processes.
--startup-check  Bounded standing/stop/exit check; no navigation goal.
--headless   Disable both GUI windows; automatically exit after result.
--no-rviz    Keep MuJoCo viewer only.
--scenario   U01 (or sudden-approach); no automatic seed substitution.
Ctrl+C: request latched safety stop, then reap this run's registered processes.
Detailed logs and result: run_data/demo/<unique-run-id>/
W13 remains FAILED (36/40). This fixed demonstration is not a reliability test.
HELP
    exit 0 ;;
esac
cpu_mode=false
for argument in "$@"; do
  if [[ "$argument" == --headless || "$argument" == --preflight ]]; then cpu_mode=true; fi
done
if ! $cpu_mode && [[ ! -f "$DEMO_ROOT/scripts/demo/gpu-profile.json" ]]; then
  echo '[demo] W14_BLOCKED: GUI remains locked after the host black-screen incident.' >&2
  echo '[demo] Only --preflight or --headless may run, with inherited CPU-only device isolation.' >&2
  echo "[demo] Read $DEMO_ROOT/docs/demo/cpu-recovery-plan.md" >&2
  exit 3
fi
export T1_VENV="${T1_VENV:-/home/haoyang/venvs/humanoid_mujoco}"
export T1_UPSTREAM="${T1_UPSTREAM:-/home/haoyang/robot_sim_sources/booster_deploy}"
export T1_ASSETS="${T1_ASSETS:-/home/haoyang/robot_sim_sources/booster_assets}"
export T1_LOCAL_DDS=1 T1_DDS_PORT=42127 ROS_DOMAIN_ID=37
export T1_DEMO_RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_demo_U01_$(cat /proc/sys/kernel/random/uuid)"
export T1_DEMO_DIR="$DEMO_ROOT/run_data/demo/$T1_DEMO_RUN_ID"
mkdir -p -- "$T1_DEMO_DIR"
export ROS_LOG_DIR="$T1_DEMO_DIR/ros-log"
runner_pid=''
forward_stop() {
  if [[ -n "$runner_pid" ]] && kill -0 "$runner_pid" 2>/dev/null; then
    kill -TERM "$runner_pid" 2>/dev/null || true
  fi
}
on_exit() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$runner_pid" ]] && kill -0 "$runner_pid" 2>/dev/null; then
    forward_stop
    wait "$runner_pid" || result=$?
  fi
  if (( result != 0 )); then
    echo "[demo] Exit $result; logs: $T1_DEMO_DIR" >&2
  fi
  exit "$result"
}
trap on_exit EXIT
trap forward_stop INT TERM
if ! source "$DEMO_ROOT/scripts/env.sh" >"$T1_DEMO_DIR/environment.log" 2>&1; then
  cat "$T1_DEMO_DIR/environment.log" >&2
  echo '[demo] Check docs/demo/quick-start.md and the configured venv/upstream paths.' >&2
  exit 2
fi
[[ -f "$DEMO_ROOT/install/setup.bash" ]] || {
  echo '[demo] Missing overlay. Run: source scripts/env.sh; /usr/bin/colcon build --symlink-install --base-paths src' >&2
  exit 2
}
# Explicit venv interpreter; launch's own simulation wrapper does the same.
"$T1_VENV/bin/python" "$DEMO_ROOT/scripts/demo/demo.py" "$@" &
runner_pid=$!
result=0
wait "$runner_pid" || result=$?
# A signal can interrupt Bash wait before the cooperative runner has reaped children.
if kill -0 "$runner_pid" 2>/dev/null; then
  wait "$runner_pid" || result=$?
fi
runner_pid=''
if [[ -f "$T1_DEMO_DIR/result.json" ]]; then
  if /usr/bin/python3 "$DEMO_ROOT/scripts/demo/finalize.py" "$T1_DEMO_DIR" "$result"; then
    result=0
  else
    result=$?
  fi
fi
exit "$result"

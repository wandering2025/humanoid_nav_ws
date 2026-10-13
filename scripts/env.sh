#!/usr/bin/env bash
# Source this file; it neither installs dependencies nor changes shell files.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Source this file: source /path/to/workspace/scripts/env.sh' >&2
  exit 2
fi

_t1_load_environment() {
  local workspace ros_setup
  workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return
  ros_setup="${T1_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  if [[ ! -f "$ros_setup" ]]; then
    echo "T1 environment: ROS setup does not exist: $ros_setup" >&2
    return 2
  fi
  if [[ "${T1_VENV:-}" != /* || ! -x "${T1_VENV:-}/bin/python" || ! -f "${T1_VENV:-}/bin/activate" ]]; then
    echo 'T1 environment: set T1_VENV to an existing absolute venv directory.' >&2
    return 2
  fi
  if [[ "${T1_UPSTREAM:-}" != /* || ! -f "${T1_UPSTREAM:-}/tasks/locomotion/robots/t1/__init__.py" ]]; then
    echo 'T1 environment: set T1_UPSTREAM to the absolute booster_deploy repository path.' >&2
    return 2
  fi
  # ROS setup scripts are not nounset-safe. Restore the caller's option.
  local had_nounset=false
  [[ "$-" == *u* ]] && had_nounset=true
  set +u
  source "$ros_setup" || { $had_nounset && set -u; return 2; }
  if [[ -f "$workspace/install/setup.bash" ]]; then
    source "$workspace/install/setup.bash" || { $had_nounset && set -u; return 2; }
  fi
  source "$T1_VENV/bin/activate" || { $had_nounset && set -u; return 2; }
  export T1_WS="$workspace" T1_VENV T1_UPSTREAM
  $had_nounset && set -u
  return 0
}
_t1_load_environment
_t1_env_result=$?
unset -f _t1_load_environment
return "$_t1_env_result"

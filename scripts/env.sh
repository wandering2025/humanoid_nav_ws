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
  # Opt-in workaround verified on this WSL host; starts no background process.
  # All communicating terminals need this setting and one local discovery server.
  if [[ "${T1_LOCAL_DDS:-0}" == 1 ]]; then
    if [[ ! "${T1_DDS_PORT:-42114}" =~ ^[1-9][0-9]{3,4}$ ]] || (( ${T1_DDS_PORT:-42114} < 1024 || ${T1_DDS_PORT:-42114} > 65535 )); then
      echo 'T1_DDS_PORT must be an integer between 1024 and 65535.' >&2
      $had_nounset && set -u
      return 2
    fi
    export T1_DDS_PORT="${T1_DDS_PORT:-42114}"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
    export ROS_DISCOVERY_SERVER="TCPv4:[127.0.0.1]:${T1_DDS_PORT}"
    export ROS_SUPER_CLIENT=TRUE
  fi
  $had_nounset && set -u
  return 0
}
_t1_load_environment
_t1_env_result=$?
unset -f _t1_load_environment
return "$_t1_env_result"

#!/usr/bin/env bash
# Run one foreground discovery server for terminals using T1_LOCAL_DDS=1.
set -euo pipefail
port="${T1_DDS_PORT:-42114}"
if [[ ! "$port" =~ ^[1-9][0-9]{3,4}$ ]] || (( port < 1024 || port > 65535 )); then
  echo 'T1_DDS_PORT must be an integer between 1024 and 65535.' >&2
  exit 2
fi
if ! command -v fast-discovery-server >/dev/null; then
  echo 'Source scripts/env.sh first; fast-discovery-server must already be installed.' >&2
  exit 2
fi
exec fast-discovery-server -i 0 -t 127.0.0.1 -q "$port"

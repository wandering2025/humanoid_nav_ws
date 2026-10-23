#!/usr/bin/env bash
# CPU contract tests; uses the simulation interpreter and existing ROS environment.
set -Eeuo pipefail
TEST_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "$TEST_ROOT/scripts/env.sh"
cd "$TEST_ROOT"
"$TEST_ROOT/scripts/demo/build_rviz.sh"
"$T1_VENV/bin/python" - <<'PY'
from pathlib import Path
import importlib.util
import sys
root = Path.cwd()
sys.path[:0] = [str(root / 'src/t1_mujoco'), str(root / 'src/t1_bringup'), str(root / 'scripts')]
# Reuse Ubuntu's test runner when pytest is not installed in the simulation venv.
if importlib.util.find_spec('pytest') is None:
    sys.path.append('/usr/lib/python3/dist-packages')
import pytest
raise SystemExit(pytest.main([
    '-q', 'src/t1_mujoco/test', 'src/t1_bringup/test',
    'scripts/advanced', 'scripts/exploration', 'scripts/evaluation/tests', 'scripts/demo',
    '--ignore=scripts/demo/test_dds_teardown.py',
]))
PY

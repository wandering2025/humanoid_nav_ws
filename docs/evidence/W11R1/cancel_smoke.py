"""Run the unchanged W11 cancellation/replacement assertions into W11R1 only."""
import importlib.util
import json
import os
from pathlib import Path
import sys

import yaml

WS = Path(__file__).resolve().parents[3]
out = WS/'docs/evidence/W11R1/raw'/sys.argv[1]
out.mkdir(exist_ok=False)
os.environ.update(T1_EVAL_OUT=str(out), ROS_LOG_DIR=str(out/'raw/ros-log'))
sys.path.insert(0, str(WS/'scripts'))
from evaluation import harness

# Reuse the original assertion function with the W13 process-owning harness.
# Its constants resolve to our output directory, never the W11 original suites.
sys.modules['harness'] = harness
spec = importlib.util.spec_from_file_location('w11_cancel_assertions', WS/'docs/evidence/W11/nav_run.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
cases = yaml.safe_load(module.CASES.read_text())
with harness.Session():
    result = module.cancel_replace(cases)
remaining = result['run']['remaining_owned_pids']
contacts = result['run']['manifest'].get('contact_counts', {})
assert not remaining, remaining
assert not any(contacts.values()), contacts
trace = module.load_trace(result['run']['run_id'])
result['repair_checks'] = dict(no_fall_or_fault=not any(
    state in ('FALLEN', 'FAULT') for state in trace['state']),
    no_contacts=not any(contacts.values()), no_remaining_processes=not remaining)
result['verdict'] = 'PASS' if result['verdict'] == 'PASS' and all(result['repair_checks'].values()) else 'FAIL'
(out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
raise SystemExit(0 if result['verdict'] == 'PASS' else 1)

"""Run the unchanged W11 cancellation/replacement assertions into W11R1 only."""
import importlib.util
import copy
import json
import os
from pathlib import Path
import sys

import yaml

WS = Path(__file__).resolve().parents[3]
audit_only = sys.argv[1] == '--audit-result'
source_result = Path(sys.argv[2]).resolve() if audit_only else None
out = source_result.parent if audit_only else WS/'docs/evidence/W11R1/raw'/sys.argv[1]
if not audit_only:
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
original_graph_checks = module.graph_checks


def graph_checks(closed):
    """The modern harness recorder is t1_evaluator, not w11_evaluator.

    Keep the actual saved graph untouched and check exact publishers/subscribers.
    The observer may subscribe; it may never become a command publisher.
    """
    checks = original_graph_checks(closed)
    graph = closed['graph']
    raw = graph['/t1/nav/cmd_vel_raw']
    checks.update(
        nav2_into_adapter_only=set(raw['publishers']) <= {'controller_server', 'behavior_server'}
        and set(raw['subscribers']) == {'t1_evaluator', 't1_navigation_adapter'},
        no_teleop_source=not graph['/t1/cmd_teleop']['publishers']
        and set(graph['/t1/cmd_teleop']['subscribers']) == {'t1_evaluator'},
        truth_isolated=set(graph['/t1/eval/ground_truth']['subscribers']) == {'t1_evaluator'})
    return checks


module.graph_checks = graph_checks
cases = yaml.safe_load(module.CASES.read_text())
if audit_only:
    result = json.loads(source_result.read_text())
    result['original_checks'] = dict(result['checks'])
    result['original_verdict'] = result['verdict']
    result['checks'].update(graph_checks(result['run']))
    result['audit_note'] = 'Recorder-name compatibility only; original result preserved; no physical rerun.'
    negative_controls = {}
    for topic, field, node, check in (
        ('/cmd_vel', 'publishers', 'rogue', 'final_only_collision_monitor'),
        ('/t1/cmd_teleop', 'publishers', 't1_evaluator', 'no_teleop_source'),
        ('/t1/eval/ground_truth', 'subscribers', 'controller_server', 'truth_isolated'),
        ('/t1/nav/cmd_vel_raw', 'subscribers', 'rogue', 'nav2_into_adapter_only')):
        altered = copy.deepcopy(result['run'])
        altered['graph'][topic][field].append(node)
        negative_controls[check] = not graph_checks(altered)[check]
    assert all(negative_controls.values())
    result['audit_negative_controls'] = negative_controls
else:
    session = harness.Session(out.name)
    try:
        result = module.cancel_replace(cases)
    finally:
        for run in harness.ACTIVE_RUNS:
            if run.closed_result is None:
                run.close(settle=0.)
        session.close()
remaining = result['run']['remaining_owned_pids']
contacts = result['run']['manifest'].get('contact_counts', {})
assert not remaining, remaining
assert not any(contacts.values()), contacts
trace = module.load_trace(result['run']['run_id'])
if 'replace' in result:
    result['replacement_settle'] = module.settle_analysis(
        trace, result['spec']['replacement_goal'], result['replace']['t_done'],
        dict(position_m=.30, yaw_deg=20.), 2.)
result['repair_checks'] = dict(no_fall_or_fault=not any(
    state in ('FALLEN', 'FAULT') for state in trace['state']),
    no_contacts=not any(contacts.values()), no_remaining_processes=not remaining)
result['verdict'] = 'PASS' if result['checks'] and all(result['checks'].values()) and all(result['repair_checks'].values()) else 'FAIL'
(out/('result-audited.json' if audit_only else 'result.json')).write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(dict(verdict=result['verdict'], checks=result['checks'], repair_checks=result['repair_checks'])))
raise SystemExit(0 if result['verdict'] == 'PASS' else 1)

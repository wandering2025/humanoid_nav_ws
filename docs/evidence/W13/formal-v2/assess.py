"""Read-only post-run evidence assessment; never edits the frozen result."""
import csv
import json
from pathlib import Path
import re
import sys

from audit import WS, host

ROOT = WS/'docs/evidence/W13/raw/w13-formal-v2'


def assess(output):
    r = json.loads((output/'result.json').read_text())
    s = json.loads((output/'support-exit.json').read_text())
    run = r.get('run', {})
    launch = output/'raw'/f'{r["run_id"]}.txt'
    text = launch.read_text(errors='replace')
    dds = (output/'raw'/f'{r["run_id"]}-dds.txt').read_text(errors='replace')
    starts = {int(pid):name for name,pid in re.findall(r'\[INFO\] \[([^\]]+)\]: process started with pid \[(\d+)\]',text)}
    ends = {int(pid):dict(name=name,returncode=0) for name,pid in re.findall(r'\[INFO\] \[([^\]]+)\]: process has finished cleanly \[pid (\d+)\]',text)}
    for name,pid,rc in re.findall(r'\[ERROR\] \[([^\]]+)\]: process has died \[pid (\d+), exit code (-?\d+)',text):
        ends[int(pid)] = dict(name=name,returncode=int(rc))
    children = [dict(pid=pid,name=name,returncode=ends.get(pid,{}).get('returncode')) for pid,name in starts.items()]
    escalation = [line for line in text.splitlines() if 'escalating' in line.lower() or re.search(r"sending signal '(SIGTERM|SIGKILL)'",line)]
    signals = s['signals']
    forced = [v for v in signals if v['signal'] in (9,15)]
    robot_ok = (run.get('exit_code') == 0 and len(children) == 17 and
                all(c['returncode'] == 0 for c in children) and
                any(c['name'].startswith('navigation_starter-') for c in children))
    parents = s['launch']
    aux_ok = all(v['returncode'] == 0 and v['authority_returncode'] == 0 for v in parents)
    # A goal tool's documented exit1 for an actual navigation failure is retained
    # as a failed case, and is never confused with a launch child crash.
    goal_nonzero = [c for v in parents for c in v['children'] if c['returncode'] != 0]
    host_state = host()
    owned = set(starts) | {s['dds_pid']} | {v['pid'] for v in parents}
    owned.update(v['authority_pid'] for v in parents if v['authority_pid'])
    owned.update(c['pid'] for v in parents for c in v['children'])
    remnants = [pid for pid in sorted(owned) if Path(f'/proc/{pid}').exists()]
    ds = [v for v in signals if v['target'] == s['dds_pid']]
    conditions = dict(
        expected_single_SIGINT=len(ds)==1 and ds[0]['event']=='os.kill' and ds[0]['signal']==2 and
            s['dds_returncode_before_close'] is None,
        ended_within_10s=s['close_error'] is None and s['close_end_wall']-s['close_start_wall'] < 10,
        no_SIGTERM_SIGKILL=not forced and not escalation,
        no_dds_crash_assertion_sanitizer_traceback=not bool(re.search(
            r'crash|assertion|sanitizer|traceback|segmentation|core dumped|fatal',dds,re.I)),
        port_released=host_state['ports_free']['42123'],
        no_dds_process=not Path(f'/proc/{s["dds_pid"]}').exists(),
        all_robot_launch_starter_children_normal=robot_ok,
        exact_integer_saved=type(s['dds_returncode']) is int)
    exception_ok = all(conditions.values())
    support_ok = exception_ok  # also require the same healthy lifecycle for exit0
    raw = WS/'run_data'/r['run_id']
    trace = list(csv.DictReader((raw/'sim_trace.csv').open()))
    contacts = r.get('metrics',{}).get('env_contacts',{})
    safety = dict(environment=contacts.get('static_env'),dynamic_contact=contacts.get('obstacle'),
                  nonfoot_ground=contacts.get('ground_nonfoot'),
                  fall=any(row['state']=='FALLEN' for row in trace),
                  fault=any(row['state']=='FAULT' for row in trace))
    safe = all(v == 0 for v in safety.values())
    checks = dict(robot_children_exit0=robot_ok, auxiliary_exit0=aux_ok, support_lifecycle=support_ok,
                  no_forced_signals=not forced and not escalation, no_traceback='Traceback' not in text,
                  no_owned_remnants=not remnants, host_clean=host_state['passed'],
                  safety_zero=safe, evidence_complete='metric_error' not in r and bool(r.get('manifest')),
                  not_INVALID=r.get('outcome') != 'INVALID')
    value = dict(run_id=r['run_id'],case=r['case_spec']['id'],outcome=r['outcome'],
                 head=r.get('manifest',{}).get('code_sha'), checks=checks,
                 continue_safe=all(checks.values()), dds_returncode=s['dds_returncode'],
                 dds_exception_conditions=conditions, dds_exception_applies=s['dds_returncode'] != 0 and exception_ok,
                 dds_status='INFRASTRUCTURE_WARNING' if s['dds_returncode'] != 0 and exception_ok else 'NORMAL' if s['dds_returncode']==0 and support_ok else 'INFRASTRUCTURE_FAILURE',
                 safety=safety, launch_children=children, goal_tool_nonzero=goal_nonzero,
                 signals=signals, escalations=escalation, remaining_owned_pids=remnants,
                 host=host_state, raw_output=str(output),simulator_output=str(raw),
                 streams='frozen harness combines stderr into stdout for launch/DDS/goal; console likewise combined')
    (output/'assessment.json').write_text(json.dumps(value,indent=2)+'\n')
    return value


if __name__ == '__main__':
    state=json.loads((ROOT/'state.json').read_text())
    attempts=[c['attempts'][-1] for c in state['cases'] if c.get('attempts')]
    value=assess(Path(attempts[-1]['output']))
    print(json.dumps({k:value[k] for k in ('run_id','case','outcome','continue_safe','dds_returncode','dds_status','safety')}),flush=True)
    if not value['continue_safe']:
        print(json.dumps(value['checks']),flush=True)
    raise SystemExit(0 if value['continue_safe'] else 1)

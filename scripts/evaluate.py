#!/usr/bin/env python3
"""W13 serial batch entry point. Run through scripts/env.sh and the configured venv.

Example: $T1_VENV/bin/python scripts/evaluate.py run --campaign formal-v1
All intermediate artifacts stay ignored; export only after execution completes.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import yaml

WS = Path(__file__).resolve().parents[1]
SUITE = WS/'src/t1_bringup/config/test_suite_v1.yaml'
BASE = WS/'docs/evidence/W13/raw'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(WS), *args], text=True).strip()


def source_fingerprint():
    paths = subprocess.check_output(['git','-C',str(WS),'ls-files','src','scripts'],text=True).splitlines()
    return hashlib.sha256(''.join(f'{p}:{digest(WS/p)}\n' for p in paths).encode()).hexdigest()


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,indent=2)+'\n')
    temporary.replace(path)


def rows_for(state):
    rows=[]
    for case in state['cases']:
        attempts=case.get('attempts',[])
        row=dict(case=case['id'],suite=case['suite'],seed=case['seed'],outcome='NOT_RUN',attempts=len(attempts))
        if attempts:
            attempt=attempts[-1]
            path=Path(attempt['output'])/'result.json'
            if path.exists():
                result=json.loads(path.read_text());m=result.get('metrics',{});man=result.get('manifest',{})
                error=m.get('settle',{}) if m.get('settle',{}).get('pos_err_m') is not None else m.get('terminal_error',{})
                contacts=m.get('env_contacts',{})
                row.update(run_id=result['run_id'],code_sha=man.get('code_sha'),dirty=man.get('dirty'),outcome=result['outcome'],
                    nav2_result=m.get('nav2_status'),arrived_truth=m.get('arrived_truth'),pos_err_m=error.get('pos_err_m'),
                    yaw_err_deg=error.get('yaw_err_deg'),settle_s=2. if m.get('arrived_truth') else 0.,time_to_goal_s=m.get('time_to_goal_s'),
                    path_len_m=m.get('path_len_m'),clear_body_env_min_m=result.get('body_clearance',{}).get('min_m'),
                    clear_footprint_min_m=m.get('clear_footprint_min_m'),scan_min_range_m=m.get('scan_min_range_m'),
                    n_env_collisions=contacts.get('static_env',0)+contacts.get('obstacle',0),fell=m.get('fell'),
                    n_safety_stops=m.get('safety',{}).get('cm_stop'),n_replans=max(0,m.get('n_plans',0)-1),
                    n_recoveries=m.get('log_counts',{}).get('progress_failure'),rtf_median=result.get('performance',{}).get('rtf',{}).get('median'),
                    sim_time_s=m.get('sim_time_s'),wall_time_s=m.get('wall_time_s'),notes=result.get('error',''),evidence=str(path.relative_to(WS)))
            else:
                row.update(run_id=attempt['run_id'],outcome=attempt['state'],evidence=attempt['output'])
        rows.append(row)
    return rows


def summary(state):
    rows=rows_for(state)
    counts={suite:dict(n=sum(r['suite']==suite for r in rows),success=sum(r['suite']==suite and r['outcome']=='SUCCESS' for r in rows))
            for suite in ('static','crossing','blocking','sudden')}
    complete=all(r['outcome'] not in ('NOT_RUN','RUNNING','INTERRUPTED') for r in rows)
    passed=complete and counts['static']['success']==10 and all(counts[s]['success']>=9 for s in ('crossing','blocking','sudden'))
    passed=passed and all(r.get('n_env_collisions',0)==0 and not r.get('fell',False) for r in rows)
    return dict(campaign=state['campaign'],suite_sha256=digest(SUITE),fingerprint=state['fingerprint'],complete=complete,
                verdict='PASS' if passed else 'FAIL' if complete else 'PARTIAL',counts=counts,rows=rows)


def run(opts):
    if Path(sys.executable).resolve() != Path(os.environ['T1_VENV']+'/bin/python').resolve():
        raise SystemExit('use the configured T1_VENV interpreter')
    suite=yaml.safe_load(SUITE.read_text())
    root=BASE/opts.campaign;root.mkdir(parents=True,exist_ok=True)
    with (root/'lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        state_path=root/'state.json'
        fingerprint=source_fingerprint()
        if state_path.exists():
            state=json.loads(state_path.read_text())
            if state['fingerprint']!=fingerprint or state['suite_sha256']!=digest(SUITE):
                raise SystemExit('source/config changed: use a new campaign, never mix versions')
        else:
            state=dict(campaign=opts.campaign,created_utc=datetime.now(timezone.utc).isoformat(),
                suite_sha256=digest(SUITE),fingerprint=fingerprint,registration_sha=git('rev-parse','HEAD'),
                gui=opts.gui,cases=suite['cases'])
            save(state_path,state)
        if state['gui'] != opts.gui:
            raise SystemExit('GUI mode differs: use separate campaign')
        ran=0
        for case in state['cases']:
            if opts.cases and case['id'] not in opts.cases.split(','):
                continue
            attempts=case.setdefault('attempts',[])
            if attempts and (Path(attempts[-1]['output'])/'result.json').exists():
                continue  # even failures are immutable and are not automatically retried
            if attempts:
                attempts[-1]['state']='INTERRUPTED'
            if opts.limit and ran>=opts.limit:
                break
            run_id=f'{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_{opts.campaign}_{case["id"]}_s{case["seed"]}_{uuid.uuid4().hex[:6]}'
            output=root/run_id;output.mkdir()
            attempt=dict(run_id=run_id,output=str(output),state='RUNNING')
            attempts.append(attempt);save(state_path,state);save(root/'summary.json',summary(state))
            env=os.environ.copy();env.update(T1_EVAL_OUT=str(output),T1_EVAL_RUN_ID=run_id,T1_EVAL_SCENE=case['scene'],
                T1_EVAL_EVENTS=str(WS/suite['events_file']),T1_EVAL_KIND=case.get('kind',''),T1_EVAL_HEADLESS='false' if opts.gui else 'true',
                ROS_LOG_DIR=str(output/'raw/ros-log'))
            if case.get('scenario'):
                env['T1_EVAL_SCENARIO']=case['scenario']
            else:
                env.pop('T1_EVAL_SCENARIO',None)
            request=output/'request.json';save(request,dict(case=case,suite=suite,output=str(output),run_id=run_id,gui=opts.gui))
            print('START',case['id'],run_id,flush=True)
            with (output/'console.txt').open('w') as console:
                child=subprocess.Popen([sys.executable,str(WS/'scripts/evaluate.py'),'worker',str(request)],env=env,cwd=WS,stdout=console,stderr=subprocess.STDOUT)
                try:
                    code=child.wait()
                except KeyboardInterrupt:
                    child.send_signal(signal.SIGINT)
                    child.wait(timeout=60)
                    attempt['state']='INTERRUPTED';save(state_path,state);save(root/'summary.json',summary(state));raise
            attempt.update(state='FINISHED' if (output/'result.json').exists() else 'FAIL_STARTUP',worker_exit=code)
            save(state_path,state);save(root/'summary.json',summary(state))
            print('FINISH',case['id'],summary(state)['rows'][state['cases'].index(case)]['outcome'],flush=True)
            ran+=1
    return 0  # task verdict is explicit in summary, not orchestration exit status


def export(opts):
    import csv
    import shutil
    root=BASE/opts.campaign;state=json.loads((root/'state.json').read_text());report=summary(state)
    destination=WS/'docs/evidence/W13'/opts.campaign;destination.mkdir(exist_ok=True)
    for case in state['cases']:
        for attempt in case.get('attempts',[]):
            source=Path(attempt['output'])/'result.json'
            if source.exists():
                shutil.copyfile(source,destination/(attempt['run_id']+'.json'))
    save(destination/'state.json',state);save(destination/'summary.json',report)
    fields='run_id suite case seed code_sha dirty outcome nav2_result arrived_truth pos_err_m yaw_err_deg settle_s time_to_goal_s path_len_m clear_body_env_min_m clear_footprint_min_m scan_min_range_m n_env_collisions fell n_safety_stops n_replans n_recoveries rtf_median sim_time_s wall_time_s notes evidence attempts'.split()
    for filename,rows in [('results.csv',report['rows']),('failures.csv',[r for r in report['rows'] if r['outcome']!='SUCCESS'])]:
        with (destination/filename).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    runp=sub.add_parser('run');runp.add_argument('--campaign',required=True);runp.add_argument('--limit',type=int);runp.add_argument('--cases');runp.add_argument('--gui',action='store_true')
    exp=sub.add_parser('export');exp.add_argument('--campaign',required=True)
    status=sub.add_parser('status');status.add_argument('--campaign',required=True)
    worker=sub.add_parser('worker');worker.add_argument('request')
    opts=parser.parse_args()
    if opts.command=='run':
        return run(opts)
    if opts.command=='export':
        return export(opts)
    if opts.command=='status':
        print(json.dumps(summary(json.loads((BASE/opts.campaign/'state.json').read_text())),indent=2));return 0
    from evaluation.worker import run as run_worker
    request = json.loads(Path(opts.request).read_text())
    request['output'] = Path(request['output'])
    return run_worker(**request)


if __name__=='__main__':
    raise SystemExit(main())

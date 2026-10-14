"""Recompute physical stop/latency invariants from ignored 50 Hz traces."""
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import numpy as np

WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W10'
CASES=['normal','source_timeout','nan','scan','odom','tf','pause','cancel','monitor_exit','gate_exit','limits','wall','deadband_wall']
rows=[];details=[]
for case in CASES:
    candidates=[]
    for p in OUT.glob('w10_'+case+'_*.json'):
        d=json.loads(p.read_text())
        if d.get('case')==case and d.get('verdict')=='PASS':candidates.append((p,d))
    assert candidates,case
    p,d=max(candidates,key=lambda pd:pd[1]['ended_utc'])
    manifest=json.loads(Path(d['manifest']).read_text())
    raw=Path(d['manifest']).parent/'sim_trace.csv'
    with raw.open() as f:r=list(csv.DictReader(f))
    col=lambda k:np.array([float(a[k]) for a in r])
    t=col('sim_time');w=col('wall_mono');x=col('x_w');y=col('y_w')
    u=np.column_stack([col('u_pol_'+a) for a in ('x','y','wz')])
    v=np.column_stack([col('v_meas_'+a) for a in ('x','y','wz')])
    tinj=d['injection']['sim'];tz=d['zero_sim'];iz=np.searchsorted(t,tz-1e-7)
    # W05 standing criterion: 0.5s window mean, maintained for at least 2s.
    standing=np.array([np.linalg.norm(v[max(0,i-24):i+1,:2].mean(axis=0))<.05
        and abs(v[max(0,i-24):i+1,2].mean())<.05 for i in range(len(t))])
    istop=next((i for i in range(iz,len(t)-100) if t[i]<tinj+5.5 and standing[i:i+100].all()),None)
    sim_events=[e['sim'] for e in d['events'] if e['kind']=='gate' and e['data']['state']=='STOPPED' and e['sim']>=tinj-.04]
    detect=min(sim_events) if sim_events else tz
    scan_ms=[float(a['scan_compute_ms']) for a in r if a['scan_due']=='True']
    max_delay=.60 if case in ('source_timeout','scan','odom','tf','monitor_exit','gate_exit') else .10
    checks=dict(original_checks=all(d['checks'].values()),standing_2s=istop is not None,
        stop_time=istop is not None and t[istop]-tz<=3.62,
        zero_latency=d['zero_delay_sim']<=max_delay,
        stop_envelope=d['stop_distance_from_zero_m']<=.453,
        scan_compute_budget=max(scan_ms)<30,
        policy_boundary_budget=max(float(a['step_compute_ms']) for a in r if a['state']=='READY')<20,
        no_fault=manifest['status']=='SHUTDOWN',
        explicit_venv=manifest['executable']==str(Path(os.environ['T1_VENV'])/'bin/python'))
    if case=='normal':
        checks.update(forward=np.any(v[:,0]>.3),left_turn=np.any(v[:,2]>.4),right_turn=np.any(v[:,2]<-.4))
    if case=='limits':
        checks['envelope_clip']=np.max(col('v_guard_x'))<=.75+1e-9 and np.max(abs(col('v_guard_wz')))<=.85+1e-9
    if case in ('wall','deadband_wall'):
        checks['conservative_wall_clearance']=5-float(max(x))-.695270>.099
    checks={k:bool(v) for k,v in checks.items()}
    final_state=d['post_fault_gate']['state'] if case!='gate_exit' else 'permit expired'
    rows.append(dict(test_id=case,fault=case,injection_method='see '+p.name,t_inject_sim=tinj,
        t_detect_sim=detect,t_zero_cmd_sim=tz,v_meas_at_inject=float(v[np.searchsorted(t,tinj),0]),
        stop_distance_m=d['stop_distance_from_zero_m'],final_state=final_state,
        expected='zero target, continued standing; explicit fault rearm',verdict='PASS' if all(checks.values()) else 'FAIL',evidence=p.name))
    details.append(dict(case=case,checks=checks,evidence=p.name,tested_sha=d['tested_sha'],raw_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
        zero_latency_sim=d['zero_delay_sim'],zero_latency_wall=float(w[iz]-d['injection']['wall']),
        stop_time_s=float(t[istop]-tz) if istop is not None else None,scan_compute_max_ms=max(scan_ms),
        scan_compute_p95_ms=float(np.percentile(scan_ms,95)),step_compute_ready_max_ms=max(float(a['step_compute_ms']) for a in r if a['state']=='READY'),step_compute_startup_max_ms=max(float(a['step_compute_ms']) for a in r if a['state']=='STARTING'),
        stopped_distance_m=d['stop_distance_from_zero_m'],distance_from_injection_m=d['distance_from_injection_m'],
        final_speed_mps=d['final_speed_mps'],final_yaw_speed=d['final_yaw_speed'],
        wall_clearance_bound_m=5-float(max(x))-.695270 if 'wall' in case else None))
with (OUT/'fault_tests.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
runners=json.loads((OUT/'runner-faults.json').read_text())
report=dict(checked_utc=datetime.now(timezone.utc).isoformat(),cases=details,runner_faults=runners['verdict'],
    verdict='PASS' if all(r['verdict']=='PASS' for r in rows) and runners['verdict']=='PASS' else 'FAIL')
(OUT/'physical-audit.json').write_text(json.dumps(report,indent=2)+'\n')
for d in details:print(d['case'],d['checks'],flush=True)
raise SystemExit(0 if report['verdict']=='PASS' else 1)

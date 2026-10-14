"""Final evidence, frozen repositories, source identity and owned-process audit."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess

WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W10'
def git(*args):return subprocess.check_output(['git',*args],cwd=WS,text=True).strip()
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
physical=json.loads((OUT/'physical-audit.json').read_text());modes=json.loads((OUT/'mode-regression.json').read_text())
runner=json.loads((OUT/'runner-faults.json').read_text());start=json.loads((OUT/'start-state.json').read_text())
reports=[json.loads((OUT/c['evidence']).read_text()) for c in physical['cases']]
pids=set()
for d in reports:pids.update(d['children'])
for p in (OUT/'raw').glob('w10_*.txt'):
    pids.update(int(v) for v in re.findall(r'process started with pid \[(\d+)\]',p.read_text()))
remaining=[p for p in sorted(pids) if Path('/proc/'+str(p)).exists()]
port_open=False
try:
    with socket.create_connection(('127.0.0.1',42120),timeout=.3):port_open=True
except ConnectionRefusedError:pass
upstreams={}
for name,expected in [('booster_deploy','7bb1462e2ac742eba5c01659b815e2e99fbe617f'),('booster_assets','12a97516f57469078707afb587a5f84c1cb110d1')]:
    path='/home/haoyang/robot_sim_sources/'+name
    sha=git('-C',path,'rev-parse','HEAD');dirty=git('-C',path,'status','--porcelain')
    upstreams[name]=dict(sha=sha,dirty=dirty,expected=expected)
user_hash=hashlib.sha256(subprocess.check_output(['git','diff','--','docs/start_commands.md'],cwd=WS)).hexdigest()
# The user may keep editing this file during W10; agents must never stage or commit it.
user_untouched=not git('log','--format=%H',start['head']+'..HEAD','--','docs/start_commands.md') and 'docs/start_commands.md' not in git('diff','--cached','--name-only').splitlines()
source_files=[p for p in (WS/'src').rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.xml') and '__pycache__' not in str(p)]
hashes={str(p.relative_to(WS)):digest(p) for p in source_files}
tested_sha=reports[0]['tested_sha']
# Runtime matrix was executed on one committed source version, with only documentation/evidence dirty.
source_matches=all(hashlib.sha256(subprocess.check_output(['git','show',tested_sha+':'+p],cwd=WS)).hexdigest()==h for p,h in hashes.items())
checks=dict(physical_audit=physical['verdict']=='PASS',all_13_cases=len(reports)==13,
    single_matrix_source=all(d['tested_sha']==tested_sha for d in reports),
    source_matches_tested_commit=source_matches,runner_faults=runner['verdict']=='PASS' and len(runner['cases'])==5,
    mode_regression=modes['verdict']=='PASS',
    build='4 packages finished' in (OUT/'build-final.txt').read_text(),
    unit_tests='86 tests, 0 errors, 0 failures, 0 skipped' in (OUT/'test-result-final.txt').read_text(),
    frozen_upstreams=all(d['sha']==d['expected'] and not d['dirty'] for d in upstreams.values()),
    user_file_untouched_by_agent=user_untouched,processes_stopped=not remaining,dds_port_closed=not port_open,
    same_branch=git('branch','--show-current')==start['branch'])
summary=dict(checked_utc=datetime.now(timezone.utc).isoformat(),cwd=str(WS),branch=git('branch','--show-current'),
    starting_head=start['head'],tested_sha=tested_sha,head_at_audit=git('rev-parse','HEAD'),
    checks=checks,verdict='PASS' if all(checks.values()) else 'FAIL',source_hashes=hashes,
    upstreams=upstreams,user_diff_sha256=dict(start=start['user_diff_sha256'],now=user_hash,changed_by_user_during_step=user_hash!=start['user_diff_sha256']),remaining_own_processes=remaining,audited_pids=sorted(pids),
    dds_port=42120,git_dirty=git('status','--short').splitlines(),
    command='source scripts/env.sh; $T1_VENV/bin/python docs/evidence/W10/final_checks.py',
    physical_summary=dict(max_zero_target_stop_distance_m=max(c['stopped_distance_m'] for c in physical['cases']),
        max_stop_time_s=max(c['stop_time_s'] for c in physical['cases']),
        max_zero_latency_sim_s=max(c['zero_latency_sim'] for c in physical['cases']),
        max_scan_compute_ms=max(c['scan_compute_max_ms'] for c in physical['cases']),
        max_ready_compute_ms=max(c['step_compute_ready_max_ms'] for c in physical['cases'])),
    limitation='software simulation protection, ideal truth odom/attitude, finite LiDAR; F09 is command-source cancellation interface, W11 action cancellation not implemented')
(OUT/'final-verification.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps({k:summary[k] for k in ('verdict','checks','tested_sha','physical_summary')},indent=2))
raise SystemExit(0 if summary['verdict']=='PASS' else 1)

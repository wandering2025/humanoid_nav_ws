"""W11 C4 audit: acceptance evidence, tested source identity, frozen repos, own processes."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess

WS = Path(os.environ['T1_WS'])
OUT = WS/'docs/evidence/W11'
START_HEAD = 'ffc01989207c97113db48aa7ce4b248dfa6f102a'
START_USER_DIFF = 'da7cc654bd88d2cd3595d8458b7034025bd418be32045cd8c451ae15d9b1f150'


def git(*args):
    return subprocess.check_output(['git', *args], cwd=WS, text=True).strip()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


suite = json.loads((OUT/'suite.json').read_text())
cases = [json.loads((OUT/f"w11_{r['case']}.json").read_text()) for r in suite['results']]
cancel = json.loads((OUT/'w11_cancel_replace.json').read_text())
mapping = json.loads((OUT/'map-quality.json').read_text())
localization = json.loads((OUT/'localization.json').read_text())
tested = {c['run']['manifest']['code_sha'] for c in cases + [cancel]}
tested_sha = tested.pop() if len(tested) == 1 else None
pids = set()
for p in (OUT/'raw').glob('w11_*.txt'):
    pids.update(int(v) for v in re.findall(r'process started with pid \[(\d+)\]', p.read_text(errors='replace')))
remaining = [p for p in sorted(pids) if Path(f'/proc/{p}').exists()]
port_open = False
try:
    with socket.create_connection(('127.0.0.1', 42121), timeout=.3):
        port_open = True
except OSError:
    pass
upstreams = {}
for name, expected in (('booster_deploy', '7bb1462e2ac742eba5c01659b815e2e99fbe617f'),
                       ('booster_assets', '12a97516f57469078707afb587a5f84c1cb110d1')):
    path = '/home/haoyang/robot_sim_sources/' + name
    upstreams[name] = dict(sha=git('-C', path, 'rev-parse', 'HEAD'), dirty=git('-C', path, 'status', '--porcelain'), expected=expected)
source = [p for p in (WS/'src').rglob('*') if p.is_file() and p.suffix in ('.py', '.yaml', '.xml', '.rviz', '.pgm', '.txt')
          and '__pycache__' not in str(p) and '.pytest_cache' not in str(p)]
hashes = {str(p.relative_to(WS)): digest(p) for p in source}
source_matches = tested_sha is not None and all(
    hashlib.sha256(subprocess.check_output(['git', 'show', f'{tested_sha}:{p}'], cwd=WS)).hexdigest() == h for p, h in hashes.items())
dirty_code = [f for c in cases + [cancel] for f in c['run']['manifest'].get('dirty_files') or []
              if not f.endswith('docs/start_commands.md')]
user_diff = hashlib.sha256(subprocess.check_output(['git', 'diff', '--', 'docs/start_commands.md'], cwd=WS)).hexdigest()
user_untouched = (not git('log', '--format=%H', f'{START_HEAD}..HEAD', '--', 'docs/start_commands.md') and
                  'docs/start_commands.md' not in git('diff', '--cached', '--name-only').splitlines())
m = [c['metrics'] for c in cases]
checks = dict(
    five_predefined_all_success=len(cases) == 5 and all(c['outcome'] == 'SUCCESS' for c in cases),
    nav2_succeeded_and_truth_arrived=all(x['nav2_status'] == 'SUCCEEDED' and x['arrived_truth'] for x in m),
    within_time_limit=all((x['time_to_goal_s'] or 1e9) <= 120 for x in m),
    no_env_contact_or_fall=all(not x['env_contacts'].get('static_env') and not x['env_contacts'].get('ground_nonfoot') and not x['fell'] for x in m),
    obvious_detour=any(x['detour'] and x['detour']['max_lateral_from_straight_m'] > 1.5 for x in m),
    graph_contracts=all(all(x['graph_checks'].values()) for x in m),
    parameter_contracts=all(all(c['parameter_checks'].values()) for c in cases),
    no_tf_or_scan_errors=all(x['log_counts']['tf_extrapolation'] + x['log_counts']['tf_old_data'] + x['log_counts']['tf_timeout'] == 0
                             and x['scan_empty'] == 0 for x in m),
    no_safety_stop_in_nominal_runs=all(x['safety']['cm_stop'] == 0 and x['safety']['cm_slowdown'] == 0 and x['safety']['gate_stopped'] == 0 for x in m),
    cancel_replace=cancel['verdict'] == 'PASS',
    map_quality=mapping['verdict'] == 'PASS', localization_reverified=localization['verdict'] == 'PASS',
    single_tested_commit=tested_sha is not None, source_matches_tested_commit=source_matches,
    only_user_file_dirty_during_runs=not dirty_code,
    unit_tests='108 tests, 0 errors, 0 failures, 0 skipped' in (OUT/'test-result-final.txt').read_text(),
    frozen_upstreams=all(d['sha'] == d['expected'] and not d['dirty'] for d in upstreams.values()),
    user_file_untouched_by_agent=user_untouched, processes_stopped=not remaining, dds_port_closed=not port_open,
    visual_evidence=(OUT/'static-runs.png').exists() and any(OUT.glob('w11_S04-*.png')))
summary = dict(checked_utc=datetime.now(timezone.utc).isoformat(), cwd=str(WS), branch=git('branch', '--show-current'),
               starting_head=START_HEAD, tested_sha=tested_sha, head_at_audit=git('rev-parse', 'HEAD'),
               checks=checks, verdict='PASS' if all(checks.values()) else 'FAIL',
               results=[dict(case=c['case'], outcome=c['outcome'], time_to_goal_s=x['time_to_goal_s'],
                             pos_err_m=x['settle'].get('pos_err_m'), yaw_err_deg=x['settle'].get('yaw_err_deg'),
                             terminal_retries=x['terminal_retries'], path_len_m=x['path_len_m'],
                             clear_footprint_min_m=x['clear_footprint_min_m'], clear_center_min_m=x['clear_center_min_m'])
                        for c, x in zip(cases, m)],
               cancel=dict(checks=cancel['checks'], cancel=cancel.get('cancel'), replace=cancel.get('replace')),
               upstreams=upstreams, source_hashes=hashes, remaining_own_processes=remaining, audited_pids=len(pids),
               dds_port=42121, user_diff_sha256=dict(start=START_USER_DIFF, now=user_diff),
               git_dirty=git('status', '--short').splitlines(),
               command='source scripts/env.sh; python3 docs/evidence/W11/final_checks.py')
(OUT/'final-verification.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({k: summary[k] for k in ('verdict', 'checks', 'tested_sha')}, indent=2))
raise SystemExit(0 if summary['verdict'] == 'PASS' else 1)

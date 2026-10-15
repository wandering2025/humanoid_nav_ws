"""Read-only freeze and host audit for W13 formal-v2; no ROS imports/runs."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

WS = Path('/home/haoyang/humanoid_nav_ws')
REPAIR = '166b4a15835a68eb6fd7d1e685f0cc68421f7d77'


def git(*args):
    return subprocess.check_output(['git', '-c', 'core.quotepath=false', '-C', str(WS), *args], text=True).strip()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def host():
    processes = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            argv = (p/'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
            comm = (p/'comm').read_text().strip()
        except OSError:
            continue
        if not argv[0] or Path(argv[0]).name in ('bash', 'sh', 'ps', 'timeout'):
            continue
        if (argv[0].startswith(('/opt/ros/', str(WS/'install'))) or
            comm.startswith('fast-discovery') or
            (Path(argv[0]).name.startswith('python') and any(
                a.startswith((str(WS), '/opt/ros/')) for a in argv[1:3]))):
            processes.append(dict(pid=int(p.name), comm=comm, argv=argv))
    sockets = subprocess.run(['ss', '-lntup'], capture_output=True, text=True, check=True)
    ports = {str(p): f':{p} ' not in sockets.stdout for p in (42123, 42124, 42126)}
    return dict(processes=processes, sockets=sockets.stdout, socket_errors=sockets.stderr,
                ports_free=ports, passed=not processes and all(ports.values()) and not sockets.stderr)


def freeze():
    paths = git('ls-files', 'src', 'scripts').splitlines()
    hashes = {p: digest(WS/p) for p in paths}
    expected = json.loads((WS/'docs/evidence/W11R2V/initial-audit.json').read_text())
    indexes = {}
    for name in ('W11R1/formal-v1-baseline-hashes.json', 'W11R1V/raw-artifact-index.json',
                 'W11R2/raw-artifact-index.json', 'W11R2V/raw-artifact-index.json'):
        index = json.loads((WS/'docs/evidence'/name).read_text())
        mismatches = [p for p, v in index.items()
                      if not (WS/p).exists() or digest(WS/p) != (v if isinstance(v, str) else v['sha256'])]
        indexes[name] = dict(files=len(index), mismatches=mismatches)
    changed = git('diff', '--name-only', REPAIR, 'HEAD').splitlines()
    upstream = {}
    for name, sha in [('booster_deploy', '7bb1462e2ac742eba5c01659b815e2e99fbe617f'),
                      ('booster_assets', '12a97516f57469078707afb587a5f84c1cb110d1')]:
        root = '/home/haoyang/robot_sim_sources/'+name
        actual = subprocess.check_output(['git','-C',root,'rev-parse','HEAD'],text=True).strip()
        dirty = subprocess.check_output(['git','-C',root,'status','--porcelain'],text=True).splitlines()
        upstream[name] = dict(head=actual, dirty=dirty, passed=actual == sha and not dirty)
    evaluator_diff = git('diff', '04d74f17a6d56d24d7da5ae8ff0c9a4398608daf', 'HEAD', '--', 'scripts')
    candidate_diff = git('diff', REPAIR, 'HEAD', '--', 'src', 'scripts', 'docs/architecture')
    frozen = expected['W05_W10_frozen_paths']
    safety_diff = git('diff', '04d74f17a6d56d24d7da5ae8ff0c9a4398608daf', 'HEAD', '--', *frozen)
    source_match = hashes == expected['source_hashes']
    return dict(source_hashes=hashes, source_json_fingerprint=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest(),
                evaluate_line_fingerprint=hashlib.sha256(''.join(f'{p}:{hashes[p]}\n' for p in paths).encode()).hexdigest(),
                source_matches_repair=source_match, candidate_diff=candidate_diff,
                formal_v1_evaluator_diff=evaluator_diff, frozen_safety_diff=safety_diff,
                repair_to_head_changed_paths=changed, historical_indexes=indexes, upstream=upstream,
                architecture_hashes={p:digest(WS/p) for p in git('ls-files','docs/architecture').splitlines()},
                user_document_sha256=digest(WS/'docs/start_commands.md'),
                passed=source_match and not candidate_diff and not evaluator_diff and not safety_diff and
                    all(not i['mismatches'] for i in indexes.values()) and all(i['passed'] for i in upstream.values()) and
                    all(p.startswith(('docs/evidence/', 'docs/plans/')) for p in changed))


if __name__ == '__main__':
    data = dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), cwd=str(WS),
                head=git('rev-parse','HEAD'), branch=git('branch','--show-current'),
                dirty=git('status','--porcelain').splitlines(), host=host())
    if '--host-only' not in sys.argv:
        data['freeze'] = freeze()
    data['passed'] = data['host']['passed'] and data.get('freeze',{}).get('passed',True)
    Path(sys.argv[1]).write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({k:v for k,v in data.items() if k not in ('host','freeze')}))
    print('host_pass',data['host']['passed'],'freeze_pass',data.get('freeze',{}).get('passed'))
    raise SystemExit(0 if data['passed'] else 1)

"""Installed W03 acceptance. Source scripts/env.sh first; run from any cwd.

No installation, physics, ROS daemon or viewer. Logs are small, timestamped and
include the tested HEAD and SHA256 of implementation files. Use --phase c2/c3/c4
to resume without rerunning unrelated checks.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import xml.etree.ElementTree as ET

WS = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
HEAD = subprocess.check_output(['git', '-C', str(WS), 'rev-parse', 'HEAD'], text=True).strip()
ENV = dict(os.environ, ROS_DOMAIN_ID='73', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
           ROS_LOG_DIR=str(WS / 'run_data/w03_ros_logs'))
Path(ENV['ROS_LOG_DIR']).mkdir(parents=True, exist_ok=True)
RESULTS = []


def now():
    return datetime.now(timezone.utc).isoformat()


def run(name, command, *, cwd=Path('/tmp'), expected=0, contains=None, env=None):
    start = now()
    proc = subprocess.Popen(command, cwd=cwd, env=env or ENV, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        output, _ = proc.communicate(timeout=60)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            output, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            output, _ = proc.communicate()
        output += '\nFAIL: timed out; terminated only this test process group.\n'
    record = dict(name=name, command=command, cwd=str(cwd), start_utc=start, end_utc=now(),
                  exit_code=proc.returncode, expected=expected, tested_head=HEAD,
                  ros_domain_id=ENV['ROS_DOMAIN_ID'], log=f'{name}.txt')
    record['pass'] = (proc.returncode == 0 if expected == 0 else proc.returncode != 0)
    if contains:
        record['pass'] &= contains in output
    (EVIDENCE / record['log']).write_text(json.dumps(record, sort_keys=True) + '\n' + output)
    RESULTS.append(record)
    print(f"{name}: {'PASS' if record['pass'] else 'FAIL'} (exit {proc.returncode})", flush=True)
    if not record['pass']:
        raise AssertionError(f'{name}: see {EVIDENCE / record["log"]}')
    return output


def verify_report(output, *, mode='sim_only', use_sim_time=False):
    reports = [json.loads(line.split('T1_DIAGNOSTIC ', 1)[1])
               for line in output.splitlines() if 'T1_DIAGNOSTIC ' in line]
    assert len(reports) == 1
    r = reports[0]
    venv = Path(ENV['T1_VENV'])
    assert r['executable'] == str(venv / 'bin/python') and r['prefix'] == str(venv)
    assert r['ros_distro'] == 'jazzy' and r['python'].startswith('3.12.3')
    assert r['libraries']['mujoco']['version'] == '3.15.0'
    assert r['libraries']['torch']['version'] == '2.14.1+cu130'
    assert r['libraries']['numpy']['version'] == '2.5.4'
    assert r['libraries']['rclpy']['path'].startswith('/opt/ros/jazzy/')
    assert r['mode'] == mode and r['use_sim_time'] == use_sim_time
    assert r['status'] == 'PASS' and r['physics_started'] is False
    assert (r['joint_count'], r['policy_dt'], r['decimation'], r['physics_dt']) == (23, 0.02, 10, 0.002)
    assert Path(r['upstream_path']) == Path(ENV['T1_UPSTREAM']).resolve()
    assert all(Path(r[k]).is_file() for k in ('mjcf_path', 'checkpoint_path'))
    assert 'Error creating socket' not in output and 'Operation not permitted' not in output
    return r


def c2():
    code = """import importlib, json, sys
result = {'executable': sys.executable, 'python': sys.version}
for name in ('rclpy', 'numpy', 'mujoco', 'torch', 'colcon_core', 'pytest'):
 try:
  m = importlib.import_module(name)
  result[name] = {'version': getattr(m, '__version__', 'n/a'), 'path': m.__file__}
 except ImportError as e:
  result[name] = {'missing': str(e)}
print(json.dumps(result, sort_keys=True))
"""
    for label, python in [('system', '/usr/bin/python3'), ('venv', str(Path(ENV['T1_VENV']) / 'bin/python'))]:
        run(f'c2-{label}-imports', [python, '-c', code])
    from ament_index_python.packages import get_package_prefix
    executable_dir = Path(get_package_prefix('t1_mujoco')) / 'lib/t1_mujoco'
    shebangs = {str(p): p.read_text().splitlines()[0] for p in (
        Path('/usr/bin/colcon'), executable_dir / 't1_diagnostic_python', executable_dir / 't1_diagnostic')}
    assert shebangs[str(executable_dir / 't1_diagnostic_python')] == '#!/usr/bin/python3'
    (EVIDENCE / 'c2-shebangs.json').write_text(json.dumps(shebangs, indent=2) + '\n')
    verify_report(run('c2-ros2-run', ['ros2', 'run', 't1_mujoco', 't1_diagnostic']))
    verify_report(run('c2-ros2-launch', ['ros2', 'launch', 't1_bringup', 'diagnostic.launch.py']), use_sim_time=True)


def c3():
    listed = run('c3-colcon-list', ['colcon', 'list', '--base-paths', 'src'], cwd=WS)
    assert {line.split()[0] for line in listed.splitlines()} == {
        't1_mujoco', 't1_description', 't1_navigation', 't1_bringup'}
    run('c3-colcon-test', ['colcon', 'test', '--base-paths', 'src', '--event-handlers', 'console_direct+'], cwd=WS)
    run('c3-test-result', ['colcon', 'test-result', '--all', '--verbose'], cwd=WS,
        contains='0 errors, 0 failures')
    # Inspect actual JUnit output too: a successful no-tests run is insufficient.
    junit = []
    for package in ('t1_mujoco', 't1_bringup'):
        path = WS / 'build' / package / 'pytest.xml'
        root = ET.parse(path).getroot()
        suites = list(root) if root.tag == 'testsuites' else [root]
        count = sum(int(s.attrib['tests']) for s in suites)
        assert count > 0
        assert sum(int(s.attrib.get(k, 0)) for s in suites for k in ('failures', 'errors', 'skipped')) == 0
        junit.append(dict(package=package, tests=count, path=str(path)))
    (EVIDENCE / 'c3-junit-summary.json').write_text(json.dumps(junit, indent=2) + '\n')


def c4():
    from ament_index_python.packages import get_package_share_directory
    resources = {name: Path(get_package_share_directory(name)) for name in (
        't1_mujoco', 't1_description', 't1_navigation', 't1_bringup')}
    expected = {'t1_mujoco': ['config/runtime.yaml'], 't1_description': ['config/README.md'],
                't1_navigation': ['config/README.md'],
                't1_bringup': ['launch/diagnostic.launch.py', 'launch/bringup.launch.py']}
    for name, files in expected.items():
        assert all((resources[name] / file).is_file() for file in files)
    verify_report(run('c4-bringup', ['ros2', 'launch', 't1_bringup', 'bringup.launch.py',
                                    'mode:=evaluation', 'use_sim_time:=false']),
                  mode='evaluation', use_sim_time=False)
    run('c4-invalid-upstream', ['ros2', 'run', 't1_mujoco', 't1_diagnostic', '--ros-args',
                              '-p', 'upstream_path:=/path/does/not/exist'], expected=2,
        contains='upstream_path does not exist')
    run('c4-launch-failure', ['ros2', 'launch', 't1_bringup', 'diagnostic.launch.py',
                            'upstream_path:=/path/does/not/exist'], expected=1,
        contains='t1_diagnostic exited with code 2')
    run('c4-disabled-sim', ['ros2', 'launch', 't1_bringup', 'bringup.launch.py', 'enable_sim:=true'],
        expected=1, contains='t1_sim is not implemented in W03')
    run('c4-missing-params', ['ros2', 'launch', 't1_bringup', 'diagnostic.launch.py',
                            'params_file:=/path/does/not/exist'], expected=1, contains='params_file does not exist')
    no_venv = dict(ENV)
    no_venv.pop('T1_VENV', None)
    run('c4-missing-venv', ['ros2', 'run', 't1_mujoco', 't1_diagnostic'], expected=2,
        contains='set T1_VENV', env=no_venv)
    run('c4-env-from-tmp', ['bash', '-uc', 'source "$1" && test "$-" != "${-//u/}" && test "$PWD" = /tmp && test "$VIRTUAL_ENV" = "$T1_VENV"',
                          'w03-env', str(WS / 'scripts/env.sh')])
    run('c4-baseline', [str(Path(ENV['T1_VENV']) / 'bin/python'), str(WS / 'docs/baseline/verify_baseline.py')], cwd=WS)
    cfg_probe = """import sys
from t1_mujoco.runtime import load_task_config
cfg = load_task_config(sys.argv[1])
from booster_deploy.utils.registry import get_task, list_tasks
original = get_task('t1_walk')
before = original.robot.default_joint_pos[0]
cfg.robot.default_joint_pos[0] = 99
assert original.robot.default_joint_pos[0] == before
assert set(list_tasks()) == {'t1_walk'}
print('PASS deep copy and T1-only task registration')
"""
    run('c4-upstream-copy', [str(Path(ENV['T1_VENV']) / 'bin/python'), '-c', cfg_probe, ENV['T1_UPSTREAM']])
    manifest = json.loads((WS / 'docs/evidence/W01/c2-assets.json').read_text())
    for name, repository in manifest['repositories'].items():
        sha = run('c4-' + name + '-head', ['git', '-C', repository['path'], 'rev-parse', 'HEAD']).strip()
        assert sha == repository['sha']
        assert run('c4-' + name + '-clean', ['git', '-C', repository['path'], 'status', '--porcelain']).strip() == ''
    (EVIDENCE / 'c4-resources.json').write_text(json.dumps({k: str(v) for k, v in resources.items()}, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=['all', 'c2', 'c3', 'c4'], default='all')
    args = parser.parse_args()
    started = now()
    error = None
    try:
        for phase in (['c2', 'c3', 'c4'] if args.phase == 'all' else [args.phase]):
            globals()[phase]()
    except Exception as exc:
        error = repr(exc)
        print(error, file=sys.stderr)
    finally:
        tracked = [p for p in (WS / 'src').rglob('*') if p.is_file() and '__pycache__' not in p.parts
                   and '.pytest_cache' not in p.parts]
        tracked += [WS / 'scripts/env.sh', WS / 'AGENTS.md', WS / 'README.md']
        hashes = {str(p.relative_to(WS)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(tracked)}
        summary = dict(phase=args.phase, start_utc=started, end_utc=now(), tested_head=HEAD,
                       implementation_sha256=hashes, environment={k: ENV.get(k) for k in (
                           'T1_VENV', 'T1_UPSTREAM', 'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_LOG_DIR')},
                       results=RESULTS, failure=error, passed=error is None)
        (EVIDENCE / f'verification-{args.phase}.json').write_text(json.dumps(summary, indent=2) + '\n')
    sys.exit(1 if error else 0)

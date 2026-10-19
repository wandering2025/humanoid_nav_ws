"""Offline GUI acceptance using saved windows, physical evidence and source hashes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

from PIL import Image

from report import audit as path_audit
from dynamic_report import audit as dynamic_audit

WS = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(run):
    result = json.loads((run/'final-result.json').read_text())
    paths, _, _ = path_audit(run)
    dynamic = dynamic_audit(run)
    screenshots = {}
    for stage in ('ready', 'dynamic', 'arrived'):
        screenshots[stage] = []
        for name in ('mujoco', 'rviz'):
            path = run/f'{stage}-{name}.png'
            if path.exists():
                with Image.open(path) as im:
                    size = list(im.size); im.verify()
                screenshots[stage].append(dict(path=str(path),sha256=sha(path),size=size))
    samples = [json.loads(l) for l in (run/'gpu/gpu-samples.jsonl').read_text().splitlines()]
    scans = [json.loads(l) for l in (run/'scan-filter.jsonl').read_text().splitlines()]
    maximum_age = max(s['output_age_s'] for s in scans)
    renderer = [line for line in (run/'rviz.log').read_text().splitlines()
                if line.startswith('OpenGL renderer string:')]
    source_paths = ('scripts/run_exploration_demo.sh','scripts/exploration/demo.py',
        'scripts/exploration/controller.py','scripts/exploration/scan_filter.py',
        'scripts/advanced/demo.py','src/t1_bringup/t1_bringup/exploration.py',
        'src/t1_bringup/t1_bringup/scan_history.py','src/t1_mujoco/t1_mujoco/simulation.py',
        'src/t1_mujoco/t1_mujoco/exploration_display.py','src/t1_mujoco/t1_mujoco/advanced_presentation.py',
        'src/t1_mujoco/t1_mujoco/runner.py','src/t1_mujoco/t1_mujoco/events.py',
        'src/t1_bringup/config/exploration_events.yaml','src/t1_navigation/config/exploration.rviz')
    sources = {}
    for name in source_paths:
        tested = subprocess.check_output(['git','show',f'{result["code_sha"]}:{name}'],cwd=WS)
        sources[name] = dict(sha256=sha(WS/name),
                            unchanged_since_run=hashlib.sha256(tested).hexdigest()==sha(WS/name))
    cleared = dynamic['vacated_footprint_in_final_slam']
    checks = dict(
        graphical_run=result.get('headless') is False,
        mission_success=result['status']=='SUCCESS' and result['arrived'],
        real_dynamic_response=result.get('dynamic',{}).get('verdict')=='PASS',
        all_selected_paths_observed=paths['all_selected_paths_observed'],
        graph_contract=all(result['graph_contract'].values()),
        no_contact_or_fall=not result['fell'] and not any(result['contacts'].values()),
        safe_stop=result['safe_stop']['physical_stationary'],
        both_windows_at_all_three_stages=all(len(v)==2 for v in screenshots.values()),
        actual_explorer_coordinates=result.get('display_coordinates_consistent') is True,
        expected_renderer=len(renderer)==1 and 'NVIDIA GeForce RTX 4050' in renderer[0],
        gpu_monitor_clean=bool(samples) and 'gpu_monitor_error' in result and result['gpu_monitor_error'] is None,
        mapping_latency_bounded=maximum_age<=.2,
        all_processes_exit_zero=all(v==0 for v in result['parent_exit_codes'].values())
            and all(v['exit_code']==0 for v in result['launch_process_exit_codes'].values()),
        no_processes_or_port=not result['remaining_owned_pids'] and not result['remaining_project_processes']
            and result['dds_port_released'],
        vacated_obstacle_has_no_occupied_ghost=bool(cleared) and cleared['occupied_samples']==0,
        tested_sources_unchanged=all(v['unchanged_since_run'] for v in sources.values()))
    return dict(status='PASS' if all(checks.values()) else 'FAIL',run_id=result['run_id'],
        code_sha=result['code_sha'],checks=checks,sources=sources,screenshots=screenshots,
        renderer=renderer,gpu_peak={key:max(s[key] for s in samples)
            for key in ('temperature_c','memory_mib','utilization_pct')},
        display_start=result.get('display_start'),dynamic=dynamic,
        mapping_max_output_age_s=maximum_age,
        truth_final_hold_max_error_m=result['truth_final_hold_max_error_m'],
        decision_count=paths['decision_count'],path_replay=paths['path_replay'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path); parser.add_argument('output',type=Path)
    args = parser.parse_args()
    result = audit(args.run)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],checks=result['checks']),indent=2))
    raise SystemExit(0 if result['status']=='PASS' else 1)


if __name__=='__main__':
    main()

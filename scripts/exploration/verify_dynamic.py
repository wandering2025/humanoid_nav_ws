"""Finalize W16 evidence from completed runs; never starts or controls ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

WS=Path(__file__).resolve().parents[2]


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dynamic',type=Path);parser.add_argument('control',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from report import audit
    from dynamic_report import audit as dynamic_audit
    reports=[audit(p)[0] for p in (args.dynamic,args.control)]
    results=[read(p/'final-result.json') for p in (args.dynamic,args.control)]
    physics=[read(WS/'run_data'/r['run_id']/'manifest.json') for r in results]
    encounter=dynamic_audit(args.dynamic)
    core=('scripts/exploration/controller.py','scripts/exploration/scan_filter.py',
          'scripts/exploration/demo.py','src/t1_bringup/t1_bringup/exploration.py',
          'src/t1_bringup/t1_bringup/scan_history.py','src/t1_mujoco/t1_mujoco/events.py',
          'src/t1_mujoco/t1_mujoco/simulation.py','src/t1_bringup/config/exploration_events.yaml')
    sources={}
    for name in core:
        current=(WS/name).read_bytes()
        sources[name]=dict(sha256=hashlib.sha256(current).hexdigest(),
            identical_in_runs=[subprocess.check_output(['git','show',r['code_sha']+':'+name],cwd=WS)==current for r in results])
    parameters=[]
    import yaml
    original=yaml.safe_load((WS/'src/t1_navigation/config/slam.yaml').read_text())
    for run in (args.dynamic,args.control):
        actual=yaml.safe_load((run/'slam.yaml').read_text())
        topic=actual['slam_toolbox']['ros__parameters'].pop('scan_topic')
        expected=yaml.safe_load(yaml.safe_dump(original));expected['slam_toolbox']['ros__parameters'].pop('scan_topic')
        parameters.append(topic=='/scan/mapping' and actual==expected)
    cleared=encounter['vacated_footprint_in_final_slam'] or {}
    checks=dict(dynamic_success=results[0]['status']=='SUCCESS' and results[0]['dynamic']['verdict']=='PASS',
        control_success=results[1]['status']=='SUCCESS' and results[1]['static_control'],
        real_body_and_motion=physics[0]['model_dimensions']['nmocap']==1 and results[0]['dynamic']['actual_travel_m']>=1.,
        control_has_no_body=physics[1]['model_dimensions']['nmocap']==0,
        same_task=(args.dynamic/'task.yaml').read_bytes()==(args.control/'task.yaml').read_bytes(),
        same_online_source=all(all(v['identical_in_runs']) for v in sources.values()),
        only_slam_scan_topic_changed=all(parameters),
        every_selected_path_observed=all(r['all_selected_paths_observed'] for r in reports),
        all_graph_contracts=all(all(r['graph_contract'].values()) for r in results),
        safe_arrivals=all(r['arrived'] and r['truth_final_hold_max_error_m']<=.3 and r['safe_stop']['physical_stationary'] for r in results),
        no_contact_or_fall=all(not r['fell'] and r['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0) for r in results),
        clean_exit=all(all(v==0 for v in r['parent_exit_codes'].values()) and all(v['exit_code']==0 for v in r['launch_process_exit_codes'].values()) for r in results),
        cleanup_complete=all(r['dds_port_released'] and not r['remaining_owned_pids'] and not r['remaining_project_processes'] for r in results),
        slam_has_no_occupied_ghost_at_hold=cleared.get('sample_count',0)>0 and cleared.get('occupied_samples')==0)
    output=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,sources=sources,
        runs=[dict(run_id=r['run_id'],code_sha=r['code_sha'],status=r['status']) for r in results],
        dynamic=encounter,user_goal_sha256=digest(WS/'presentation/goal.yaml'))
    args.output.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(dict(status=output['status'],checks=checks),indent=2))
    raise SystemExit(0 if all(checks.values()) else 1)


if __name__=='__main__':
    main()

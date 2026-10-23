"""Offline W16 evidence: replay observed-grid path checks and plot actual traces.

Run after the mission. This program has no ROS clients and sends no commands.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from t1_bringup.exploration import ObservedMap


def audit(run):
    result = json.loads((run/'final-result.json').read_text())
    events = [json.loads(line) for line in (run/'explorer/decisions.jsonl').read_text().splitlines()]
    decisions = [e for e in events if e['event']=='decision']
    replay = []
    for decision in decisions:
        snap = run/'explorer'/decision['snapshot']
        with np.load(snap) as data:
            grid = ObservedMap(data['data'],float(data['resolution']),data['origin'],
                               decision['pose'][:2],inflated=bool(data['inflated']))
            replay.append(dict(snapshot=decision['snapshot'],
                               sha256=hashlib.sha256(snap.read_bytes()).hexdigest(),
                               target_known_reachable=grid.is_reachable(decision['target']),
                               entire_path_known_reachable=grid.path_known(decision['path']),
                               final=decision['final']))
    observation = json.loads((run/'observation.json').read_text())
    points = [[r['mx'],r['my']] for r in observation['rows'] if r['mx'] is not None]
    return dict(run_id=result['run_id'],code_sha=result['code_sha'],status=result['status'],
                raw_path=str(run.resolve()),arrived=result['arrived'],
                error=result.get('error'),exploration=result.get('exploration'),
                truth_final_error_m=result.get('truth_final_error_m'),
                truth_final_hold_max_error_m=result.get('truth_final_hold_max_error_m'),
                contacts=result['contacts'],fell=result['fell'],safe_stop=result.get('safe_stop'),
                dynamic=result.get('dynamic'),static_control=result.get('static_control'),
                graph_contract=result['graph_contract'],infrastructure_valid=result['infrastructure_valid'],
                parent_exit_codes=result.get('parent_exit_codes'),
                launch_process_exit_codes=result.get('launch_process_exit_codes'),
                dds_port_released=result['dds_port_released'],
                remaining_owned_pids=result.get('remaining_owned_pids'),
                remaining_project_processes=result.get('remaining_project_processes'),
                scene=json.loads((run/'provenance.json').read_text())['simulator_scene'],
                decision_count=len(decisions),exploration_target_count=sum(not d['final'] for d in decisions),
                event_counts={key:sum(e['event']==key for e in events) for key in sorted({e['event'] for e in events})},
                path_replay=replay,
                all_selected_paths_observed=all(r['target_known_reachable'] and r['entire_path_known_reachable'] for r in replay),
                trajectory_y_range=[min(p[1] for p in points),max(p[1] for p in points)] if points else None,
                trajectory_length_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()) if len(points)>1 else 0.), decisions, points


def plot(run, report, decisions, points, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import yaml
    from PIL import Image
    metadata = yaml.safe_load((run/'maps/final.yaml').read_text())
    pixels = np.asarray(Image.open(run/'maps'/metadata['image']))
    x,y,_ = metadata['origin']; res = metadata['resolution']
    fig,axis = plt.subplots(figsize=(9,7))
    axis.imshow(pixels,cmap='gray',vmin=0,vmax=254,
                extent=[x,x+pixels.shape[1]*res,y,y+pixels.shape[0]*res])
    if points:
        pts = np.asarray(points);axis.plot(pts[:,0],pts[:,1],color='#762a83',label='Actual TF trajectory',lw=2)
        axis.scatter(*pts[0],color='#2166ac',s=80,label='A (exploration start)')
    for index,d in enumerate(decisions):
        axis.scatter(*d['target'],marker='x',color='#e66101',s=50)
        axis.annotate(str(index+1),d['target'],xytext=(4,4),textcoords='offset points')
    goal = (report.get('exploration') or {}).get('target_map')
    if goal:
        axis.scatter(*goal,marker='*',s=180,color='#1b7837',label='B resolved from A + offset')
    axis.plot([],[],marker='x',ls='',color='#e66101',label='Online decisions (numbered)')
    axis.set_aspect('equal');axis.set_xlabel('map x [m]');axis.set_ylabel('map y [m]')
    axis.set_title(f"W16 {report['status']} | {report['exploration_target_count']} exploration targets\n"
                   'Live SLAM map and measured trajectory; no predefined route')
    axis.legend(loc='best');fig.tight_layout();fig.savefig(output,dpi=150);plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    report,decisions,points=audit(args.run)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    plot(args.run,report,decisions,points,args.output.with_suffix('.png'))
    print(json.dumps({k:report[k] for k in ('status','arrived','decision_count','all_selected_paths_observed','trajectory_y_range')}))


if __name__=='__main__':
    main()

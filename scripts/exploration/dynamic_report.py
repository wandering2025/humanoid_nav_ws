"""Independent post-run audit of observed dynamic responses; no ROS access."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


def read(path):
    return json.loads(path.read_text())


def audit(run):
    result = read(run/'final-result.json')
    observation = read(run/'observation.json')
    filters = [json.loads(l) for l in (run/'scan-filter.jsonl').read_text().splitlines()]
    by_stamp = {round(r['sim'],4): r for r in filters}
    active = [s for s in observation['dynamic_scans'] if s['obstacle']['phase'] not in ('staged','finished')]
    paired = [(s,by_stamp[round(s['sim'],4)]) for s in active if round(s['sim'],4) in by_stamp]
    hits = sum(len(s['hits']) for s,_ in paired)
    removed = sum(sum(h[0] in f['masked_indices'] for h in s['hits']) for s,f in paired)
    moving = [o for o in observation['obstacles'] if o['phase'].startswith('move_')]
    held = [o for o in observation['obstacles'] if o['phase'].startswith('hold_')]
    all_sim = [f['sim'] for f in filters]
    stats = dict(scan_frames=len(filters), active_scan_pairs=len(paired),
        obstacle_hit_rays=hits, obstacle_hit_rays_withheld_from_slam=removed,
        withheld_fraction=removed/hits if hits else None,
        filtered_frames=sum(bool(f['masked_indices']) for f in filters),
        max_output_gap_s=max(np.diff(all_sim)) if len(all_sim)>1 else None,
        processing_ms_p50=float(np.median([f['processing_ms'] for f in filters])),
        processing_ms_p99=float(np.percentile([f['processing_ms'] for f in filters],99)),
        moving_samples=len(moving), holding_samples=len(held),
        filter_sha256=hashlib.sha256((run/'scan-filter.jsonl').read_bytes()).hexdigest())
    # Read actual saved SLAM pixels at the observed holding body's former
    # footprint. Geometry is evaluation-only and is never fed to the robot.
    cleared = None
    if held and (run/'maps/final.yaml').exists():
        from PIL import Image
        meta = read(run/'maps/final.meta.json')
        tf = meta['map_to_odom_at_save']; q = tf['quaternion']
        heading = math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
        c,s = math.cos(heading),math.sin(heading)
        mapping = yaml.safe_load((run/'maps/final.yaml').read_text())
        pixels = np.asarray(Image.open(run/'maps'/mapping['image']))
        ox,oy,_ = mapping['origin']; res=mapping['resolution']
        body=held[len(held)//2]; x,y,_=body['position']; sx,sy,_=body['size']
        values=[]
        for a in np.arange(x-sx/2+.05,x+sx/2,.05):
            for b in np.arange(y-sy/2+.05,y+sy/2,.05):
                mx,my = tf['x']+c*a-s*b,tf['y']+s*a+c*b
                ix,iy=math.floor((mx-ox)/res),math.floor((my-oy)/res)
                if 0<=iy<pixels.shape[0] and 0<=ix<pixels.shape[1]:
                    values.append(int(pixels[pixels.shape[0]-1-iy,ix]))
        cleared=dict(observed_hold_position_odom=body['position'],sample_count=len(values),
            occupied_samples=sum(v<65 for v in values),unknown_samples=sum(65<=v<250 for v in values),
            free_samples=sum(v>=250 for v in values))
    return dict(run_id=result['run_id'],code_sha=result['code_sha'],status=result['status'],
                dynamic=result.get('dynamic'),filter=stats,vacated_footprint_in_final_slam=cleared,
                graph=result['graph_contract'],static_control=result.get('static_control'))


def plot(run, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    obs=read(run/'observation.json'); obstacles=obs['obstacles']; rows=obs['rows']
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    if obstacles:
        axes[0].plot([r['sim_time'] for r in obstacles],[r['position'][0] for r in obstacles],label='Measured obstacle x')
        active=[o['sim_time'] for o in obstacles if o['phase'] not in ('staged','finished')]
        if active:
            for ax in axes:ax.axvspan(min(active),max(active),color='#fdb863',alpha=.25)
    axes[0].set_ylabel('Obstacle x [m]');axes[0].legend()
    times=[];speed=[]
    for i in range(5,len(rows)):
        a,b=rows[i-5],rows[i];dt=b['sim']-a['sim']
        if dt>0:times.append(b['sim']);speed.append(math.hypot(b['x']-a['x'],b['y']-a['y'])/dt)
    axes[1].plot(times,speed,label='Measured body speed (0.5 s)')
    axes[1].plot([r['sim'] for r in rows],[r['final'][0] if r.get('final') else np.nan for r in rows],label='Final command vx',alpha=.7)
    axes[1].axhline(.05,color='gray',ls=':');axes[1].set_ylabel('m/s');axes[1].legend()
    scans=obs['dynamic_scans'];costs=obs['cost_samples']
    axes[2].plot([s['sim'] for s in scans],[len(s['hits']) for s in scans],label='Raw scan obstacle hits')
    axes[2].plot([s['sim'] for s in costs],[s['max_cost'] if s['max_cost'] is not None else np.nan for s in costs],label='Live local costmap maximum',alpha=.7)
    axes[2].set_xlabel('Simulation time [s]');axes[2].legend()
    fig.suptitle('W16 physical dynamic obstacle encounter (shaded interval)')
    fig.tight_layout();fig.savefig(output,dpi=150);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args(); report=audit(args.run)
    args.output.write_text(json.dumps(report,indent=2)+'\n');plot(args.run,args.output.with_suffix('.png'))
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()

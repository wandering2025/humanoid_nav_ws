"""Export reproducible trajectories and command/physical-stop traces."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

WS=Path(__file__).resolve().parents[3];OUT=WS/'docs/evidence/W12'
layout=yaml.safe_load((WS/'src/t1_mujoco/worlds/dynamic_crossing_world.yaml').read_text())
fig,axes=plt.subplots(2,3,figsize=(16,9))
for col,kind in enumerate(('crossing','blocking','approach')):
    index=3 if kind=='blocking' else 1
    label=f'w12_smoke_{kind}_{index}';r=json.loads((OUT/f'{label}.json').read_text())
    trace=np.genfromtxt(OUT/f'{label}-trajectory.csv',delimiter=',',names=True)
    obs=np.genfromtxt(OUT/f'{label}-obstacle.csv',delimiter=',',names=True)
    ax=axes[0,col];b=layout['bounds']
    ax.plot([b[0],b[1],b[1],b[0],b[0]],[b[2],b[2],b[3],b[3],b[2]],color='black',lw=1)
    for box in layout['boxes']:
        x,y,_=box['center'];sx,sy,_=box['size']
        ax.add_patch(plt.Rectangle((x-sx/2,y-sy/2),sx,sy,color='gray'))
    successes=0
    for trial in range(1,4):
        stem=f'w12_smoke_{kind}_{trial}'
        item=json.loads((OUT/f'{stem}.json').read_text())
        track=np.genfromtxt(OUT/f'{stem}-trajectory.csv',delimiter=',',names=True)
        successes += item['outcome']=='SUCCESS'
        ax.plot(track['x_w'],track['y_w'],lw=1.2,alpha=.8,label=f'{trial}: {item["outcome"]}')
        ax.scatter(track['x_w'][-1],track['y_w'][-1],s=25,marker='o' if item['outcome']=='SUCCESS' else 'x')
    ax.plot(obs['x'],obs['y'],'--',color='tab:red',label='Kinematic obstacle')
    target=(6 if kind=='approach' else 4.5 if kind=='blocking' else 2.75)
    sx,sy,_=r['dynamic']['scan_topic_samples'][0]['obstacle']['size']
    ax.add_patch(plt.Rectangle((target-sx/2,-sy/2),sx,sy,fill=False,color='tab:red',lw=2))
    ax.scatter([0,9],[0,0],c=['green','red'],s=35)
    ax.set(xlabel='world x (m)',ylabel='world y (m)',title=f"{kind}: {successes}/3 complete successes")
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    ax=axes[1,col];t=trace['sim_time']-r['nav2']['accepted_sim']
    ax.plot(t,trace['cmd_vel_x'],label='CM final vx')
    # .5s position-window speed, independent of instantaneous gait oscillation.
    speed=np.full(len(t),np.nan)
    speed[5:]=np.hypot(trace['x_w'][5:]-trace['x_w'][:-5],trace['y_w'][5:]-trace['y_w'][:-5])/.5
    ax.plot(t,speed,label='Measured .5s speed')
    trigger=r['dynamic']['trigger_sim']-r['nav2']['accepted_sim']
    ax.axvline(trigger,color='tab:red',ls=':',label='Scene triggered')
    if r['dynamic']['actual_stop_sim'] is not None:
        ax.axvline(r['dynamic']['actual_stop_sim']-r['nav2']['accepted_sim'],color='gray',ls='--',label='Physical stop')
    if r['dynamic']['resume_sim'] is not None:
        ax.axvline(r['dynamic']['resume_sim']-r['nav2']['accepted_sim'],color='green',ls='--',label='Safe resume')
    ax.set(xlabel='sim seconds after goal',ylabel='m/s',ylim=(-.03,.65),title=f'Commands / physical speed: case {index}');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('W12 reactive dynamic navigation: LiDAR/costmaps, no future obstacle trajectory input')
fig.tight_layout();fig.savefig(OUT/'dynamic-runs.png',dpi=150)

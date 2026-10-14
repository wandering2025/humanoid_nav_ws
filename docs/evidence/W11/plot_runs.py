"""Plot the W11 predefined runs on the saved SLAM map (truth trajectories, first plans)."""
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WS = Path(__file__).resolve().parents[3]
OUT = WS/'docs/evidence/W11'
MAP = WS/'src/t1_navigation/maps/static_world'
config = yaml.safe_load(MAP.with_suffix('.yaml').read_text())
grid = np.flipud(np.array(Image.open(MAP.parent/config['image'])))
iy, ix = np.where(grid < 65)
xy = np.array(config['origin'][:2]) + np.c_[ix + .5, iy + .5]*config['resolution']
cases = yaml.safe_load((WS/'src/t1_navigation/config/w11_static_cases.yaml').read_text())['cases']
fig, ax = plt.subplots(figsize=(11, 8.5))
ax.scatter(xy[:, 0], xy[:, 1], s=1.5, c='black', label='SLAM occupied cells (saved map)')
colors = plt.cm.tab10(np.arange(len(cases)))
for color, case in zip(colors, cases):
    label = f"w11_{case['id']}"
    result = json.loads((OUT/f'{label}.json').read_text())
    traj = np.loadtxt(OUT/f'{label}-trajectory.csv', delimiter=',', skiprows=1)
    ax.plot(traj[:, 1], traj[:, 2], color=color, lw=1.8,
            label=f"{case['id']} {case['kind']}: {result['outcome']}, {result['metrics']['time_to_goal_s'] or float('nan'):.1f} s")
    if result.get('plans'):
        plan = np.array(result['plans'][0]['points'])
        ax.plot(plan[:, 0], plan[:, 1], color=color, lw=.8, ls='--', alpha=.7)
    for pose, marker in ((case['start'], 'o'), (case['goal'], '*')):
        ax.scatter([pose[0]], [pose[1]], color=color, marker=marker, s=90, edgecolor='k', zorder=5)
        ax.arrow(pose[0], pose[1], .45*math.cos(pose[2]), .45*math.sin(pose[2]), color=color, width=.03, zorder=5)
ax.set_aspect('equal')
ax.set(xlabel='map x (m)', ylabel='map y (m)',
       title='W11 static predefined runs: truth trajectories (solid), first global plan (dashed)\n'
             'o start, * goal (arrows = yaw); robot centre kept >=2.05 m from surfaces')
ax.legend(loc='lower center', fontsize=8, ncol=2, bbox_to_anchor=(.5, -.32))
ax.grid(alpha=.2)
fig.tight_layout()
fig.savefig(OUT/'static-runs.png', dpi=140)
print('wrote', OUT/'static-runs.png')

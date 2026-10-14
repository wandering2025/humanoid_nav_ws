"""Standalone metric figure from measured raycast scans; system matplotlib."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).parent
x=np.load(OUT/'raw/wall-points.npz')
fig,axs=plt.subplots(1,3,figsize=(12,4),layout='constrained')
for ax,phases,title in zip(axs,[['standing'],['forward','rotated_forward'],['turn90','stop2']],['Standing sway','Physical walking','Physical turn and settling']):
    for phase in phases:
        points=x['xy'][x['phase']==phase]
        ax.scatter(points[:,0],points[:,1],s=.15,label=phase,alpha=.4)
    ax.plot([-2,8,8,-2,-2],[-5,-5,5,5,-5],'k--',lw=.8,label='actual wall face')
    ax.set(title=title,xlabel='odom x (m)',ylabel='odom y (m)',aspect='equal',xlim=(-2.2,8.2),ylim=(-5.2,5.2));ax.legend(fontsize=7)
fig.suptitle('W08: body-fixed 9-layer rays → attitude compensation → 1° LaserScan bins')
fig.savefig(OUT/'wall-overlay.png',dpi=150)

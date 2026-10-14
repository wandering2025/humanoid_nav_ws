"""Metric top views derived from the exact layout/footprint configuration."""
from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon
import yaml
WS=Path(__file__).resolve().parents[3];OUT=WS/'docs/evidence/W07'
fp=yaml.safe_load((WS/'src/t1_description/config/footprint.yaml').read_text())['polygon']
for path in sorted((WS/'src/t1_mujoco/worlds').glob('*.yaml')):
    s=yaml.safe_load(path.read_text());fig,ax=plt.subplots(figsize=(8,8))
    x0,x1,y0,y1=s['bounds'];ax.add_patch(Rectangle((x0-.1,y0-.1),x1-x0+.2,y1-y0+.2,fill=False,linewidth=9,color='#685744'))
    for b in s['boxes']:
        x,y,_=b['center'];sx,sy,_=b['size'];ax.add_patch(Rectangle((x-sx/2,y-sy/2),sx,sy,color='#95816a'));ax.text(x,y,b['name'],ha='center',va='center',fontsize=7)
    for r in s['activity_regions']:
        a,b,c,d=r['bounds'];ax.add_patch(Rectangle((a,c),b-a,d-c,facecolor='#91b7d333',edgecolor='#3476a4',hatch='//'));ax.text((a+b)/2,(c+d)/2,r['name'],color='#225477',ha='center')
    a=s['initial_pose'];b=s['goal'];ax.add_patch(Polygon([[x+a[0],y+a[1]] for x,y in fp],fill=False,edgecolor='green'))
    ax.scatter([a[0],b[0]],[a[1],b[1]],c=['green','red'],s=60);ax.text(a[0],a[1]-.8,'A / yaw=0',ha='center');ax.text(b[0],b[1]-.8,'B / yaw=0',ha='center');ax.arrow(a[0],a[1],.8,0,width=.03,color='green')
    ax.set(xlim=(x0-.6,x1+.6),ylim=(y0-.6,y1+.6),xlabel='world X (m)',ylabel='world Y (m)',title=s['name']+' — static layout\nWalls/boxes: height 1.5 m; hatched: reserved activity regions')
    if s['name']=='stress_test_world':ax.annotate('1.90 m gate',xy=(3,0),xytext=(1,-1.7),arrowprops=dict(arrowstyle='->'))
    ax.set_aspect('equal');ax.set_xticks(range(-2,9));ax.set_yticks(range(-5,6));ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/(s['name']+'-top.png'),dpi=130);plt.close(fig)

import sys
sys.path.insert(0,'scripts/demo');from cpu_guard import enter;enter()
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pathlib import Path
import json,math
w=Path('.');out=Path(sys.argv[1]) if len(sys.argv)>1 else w/'docs/evidence/advanced-optimization';out.mkdir(parents=True,exist_ok=True)
old='20261015T180213Z_advanced_c20b40f2-12f5-4b5c-8c96-0d5a68461b8f'
new=sys.argv[2]
fig,axes=plt.subplots(1,2,figsize=(14,6),layout='constrained')
for ax,ident,title,b in zip(axes,(old,new),('Published v0.2.0: return-to-A loop','Optimized: continuous mapping, then B'),((14,0),(12,0))):
 r=json.loads((w/'run_data/advanced'/ident/'final-result.json').read_text());o=json.loads((w/'run_data/advanced'/ident/'observation.json').read_text());rows=o['rows'];samples=[]
 for v in rows:
  if not samples or v['sim']-samples[-1]['sim']>=.5:samples.append(v)
 dist=sum(math.hypot(v['x']-u['x'],v['y']-u['y']) for u,v in zip(samples,samples[1:]))
 ax.add_patch(Rectangle((-4,-4),22,16,fill=False,color='#424c55',lw=2))
 ax.add_patch(Rectangle((7,-4),1,8.5,color='#576574'))
 ax.plot([v['x'] for v in samples],[v['y'] for v in samples],c='#7b3db9',lw=1.4,label='Measured robot trajectory')
 if ident==new:
  points=r['a_to_b_overview']['points'];ax.plot([p[0] for p in points],[p[1] for p in points],c='#20a6ae',lw=1.1,alpha=.8,label='NavFn A-B overview (display only)')
  active=[v for v in o['obstacles'] if v['phase'] not in ('staged','finished')]
  mid=next(v for v in active if v['phase']=='hold_0');x,y,_=mid['position'];sx,sy,_=mid['size'];ax.add_patch(Rectangle((x-sx/2,y-sy/2),sx,sy,color='#ed8927',label='Cart during wait (withdrawn before passage)'))
  t=(active[0]['sim_time']+active[-1]['sim_time'])/2;v=min(rows,key=lambda v:abs(v['sim']-t));ax.annotate(f"{r['dynamic']['blocked_wait']['stationary_s']:.1f} s measured wait",xy=(v['x'],v['y']),xytext=(1,2),arrowprops={'arrowstyle':'->'},fontsize=10)
 ax.scatter([0,b[0]],[0,b[1]],c=['#148fc0','#218c53'],s=55,zorder=5);ax.text(.3,-.6,'A');ax.text(b[0]+.3,b[1]-.6,'B')
 ax.set(xlim=(-4.8,18.8),ylim=(-4.8,13),aspect='equal',xlabel='x (m)',ylabel='y (m)',title=f'{title}\n{dist:.1f} m sampled travel | {rows[-1]["sim"]-rows[0]["sim"]:.1f} sim s')
 ax.legend(loc='upper left',fontsize=8);ax.grid(alpha=.12)
fig.suptitle('Actual simulation evidence — different mission workflows and B locations; not an algorithm speed benchmark',fontsize=11)
fig.savefig(out/'trajectory-comparison.png',dpi=160)
fig.savefig(out/'trajectory-comparison.svg')
print(out/'trajectory-comparison.png')

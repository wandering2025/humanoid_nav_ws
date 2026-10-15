from pathlib import Path
import sys,json
sys.path.insert(0,'scripts/demo');from cpu_guard import enter;enter()
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2,yaml
run=Path(sys.argv[1]);out=Path(sys.argv[2]) if len(sys.argv)>2 else Path('docs/evidence/advanced-optimization');out.mkdir(parents=True,exist_ok=True)
fig,axes=plt.subplots(1,3,figsize=(14,4.6),layout='constrained')
for ax,label,title in zip(axes,('initial','mapped','final'),('A: initial observation','After autonomous mapping','B: final observed map')):
 p=run/'maps'/f'{label}.pgm';m=yaml.safe_load(p.with_suffix('.yaml').read_text());im=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE);h,w=im.shape;res=m['resolution'];ox,oy=m['origin'][:2]
 meta=json.loads(p.with_suffix('.meta.json').read_text());ax.set_facecolor('#cdcdcd');ax.imshow(im,cmap='gray',vmin=0,vmax=254,extent=[ox,ox+w*res,oy,oy+h*res],origin='upper')
 ax.scatter([0,12],[0,0],s=20,c=['#148fc0','#218c53']);ax.text(.3,.3,'A');ax.text(12.3,.3,'B')
 ax.set(xlim=(-4.5,18.5),ylim=(-4.5,12.5),aspect='equal',xlabel='x (m)',ylabel='y (m)',title=f'{title}\nsim {meta["sim"]:.1f}s')
fig.suptitle('Actual SLAM occupancy grids: white free, black occupied, grey unknown (offline evidence, not an RViz capture)',fontsize=10)
fig.savefig(out/'map-growth.png',dpi=150)

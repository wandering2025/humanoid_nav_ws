from pathlib import Path
import sys,json,hashlib,cv2,numpy as np,yaml
run=Path(sys.argv[1]);reports={}
for label in ('initial','mapped','final'):
 p=run/'maps'/f'{label}.pgm';m=yaml.safe_load(p.with_suffix('.yaml').read_text());im=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE);h,w=im.shape;res=m['resolution'];ox,oy=m['origin'][:2]
 xx=ox+(np.arange(w)+.5)*res;yy=oy+(h-np.arange(h)-.5)*res;x,y=np.meshgrid(xx,yy)
 known=im!=205;occupied=im==0;regions={}
 for name,box in {'west':[-2,6,-2,10],'east':[9,16,-2,10],'north_gallery':[6,10,6,10]}.items():
  a,b,c,d=box;eps=res*1e-7;select=(x>=a-eps)&(x<b-eps)&(y>=c-eps)&(y<d-eps)
  expected=round((b-a)*(d-c)/res**2)
  regions[name]=dict(bounds=box,expected_cells=expected,cells_within_saved_grid=int(select.sum()),known_fraction=float(known[select].sum()/expected),free_fraction=float((im[select]>=250).sum()/expected))
 assert all(0<=v['known_fraction']<=1.000001 for v in regions.values()),regions
 report=dict(source=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),resolution=res,known_m2=float(known.sum()*res**2),occupied_m2=float(occupied.sum()*res**2),regions=regions)
 if label=='final':
  # Evaluation only: never loaded into SLAM, planners or safety. Compare actual
  # occupied cell centres to known physical structure lines in the design.
  points=np.column_stack((x[occupied],y[occupied]));lines={
   'west_wall':np.column_stack((np.full(75,-4.),np.linspace(-3.8,11.8,75))),
   'east_wall':np.column_stack((np.full(75,18.),np.linspace(-3.8,11.8,75))),
   'north_wall':np.column_stack((np.linspace(-3.8,17.8,100),np.full(100,12.))),
   'south_wall':np.column_stack((np.r_[np.linspace(-3.8,6.8,50),np.linspace(8.2,17.8,50)],np.full(100,-4.))),
   'partition_west':np.column_stack((np.full(40,7.),np.linspace(-3.8,4.4,40))),
   'partition_east':np.column_stack((np.full(40,8.),np.linspace(-3.8,4.4,40))),
   'partition_end':np.column_stack((np.linspace(7.,8.,10),np.full(10,4.5)))}
  report['structural_distance_m']={}
  for name,samples in lines.items():
   distances=np.sqrt(((samples[:,None,:]-points[None,:,:])**2).sum(axis=2)).min(axis=1)
   report['structural_distance_m'][name]=dict(mean=float(distances.mean()),p95=float(np.quantile(distances,.95)),maximum=float(distances.max()))
 reports[label]=report
reports['scope']='Offline structure/coverage description from real SLAM PGM; no analytic map was provided to navigation. Half-open regions use cell centres; saved-grid coverage outside the ROI is excluded and missing cells count unknown. Known area includes occupied cells; finite-sample map-quality evidence, not sensor realism proof.'
(run/'map-coverage.json').write_text(json.dumps(reports,indent=2,allow_nan=False)+'\n')
print(json.dumps(reports,indent=2,allow_nan=False))

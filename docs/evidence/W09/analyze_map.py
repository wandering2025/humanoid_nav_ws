"""Evaluate a SLAM-saved grid against the scene; never generate a map from geometry."""
import json
import hashlib
from pathlib import Path
import math
import numpy as np
import yaml
from PIL import Image
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WS=Path(__file__).resolve().parents[3]
OUT=WS/'docs/evidence/W09'
MAP=WS/'src/t1_navigation/maps/static_world'
result=json.loads((OUT/'mapping.json').read_text())
assert result['verdict']=='PASS', result.get('error')
run=result['run']; manifest=run['manifest']
config=yaml.safe_load(MAP.with_suffix('.yaml').read_text())
pixels=np.array(Image.open(MAP.parent/config['image']))
resolution=config['resolution'];origin=np.array(config['origin'])
grid=np.flipud(pixels)
occupied=grid<65
iy,ix=np.where(occupied)
xy=origin[:2]+np.c_[ix+.5,iy+.5]*resolution
# First map->odom is the anchor; odom is the world horizontal projection.
tx,ty,a=result['initial_map_odom'];c,s=math.cos(a),math.sin(a)
R=np.array([[c,-s],[s,c]])
world=(xy-np.array([tx,ty]))@R
world_map_xy=-R.T@np.array([tx,ty])
scene=manifest['scene_provenance']['scene_metadata']
bounds=scene['bounds']
rectangles=[(bounds[0]-.2,bounds[0],bounds[2],bounds[3]),
            (bounds[1],bounds[1]+.2,bounds[2],bounds[3]),
            (bounds[0],bounds[1],bounds[2]-.2,bounds[2]),
            (bounds[0],bounds[1],bounds[3],bounds[3]+.2)]
for box in scene['boxes']:
    x,y,_=box['center'];w,h,_=box['size'];rectangles.append((x-w/2,x+w/2,y-h/2,y+h/2))
def surface_distance(points,rect):
    x0,x1,y0,y1=rect
    dx=np.maximum.reduce([x0-points[:,0],points[:,0]-x1,np.zeros(len(points))])
    dy=np.maximum.reduce([y0-points[:,1],points[:,1]-y1,np.zeros(len(points))])
    outside=np.hypot(dx,dy)
    inside=np.minimum.reduce([points[:,0]-x0,x1-points[:,0],points[:,1]-y0,y1-points[:,1]])
    return np.where(inside>=0,inside,outside)
errors=np.min([surface_distance(world,r) for r in rectangles],axis=0)
tree=cKDTree(world)
# Physical wall faces sampled at 5cm for completeness / gaps / ghosting evidence.
expected=[]
for x in (bounds[0],bounds[1]):
    expected.extend((x,y) for y in np.arange(bounds[2],bounds[3]+.01,.05))
for y in (bounds[2],bounds[3]):
    expected.extend((x,y) for x in np.arange(bounds[0],bounds[1]+.01,.05))
for r in rectangles[4:]:
    x0,x1,y0,y1=r
    for x in (x0,x1):
        expected.extend((x,y) for y in np.arange(y0,y1+.01,.05))
    for y in (y0,y1):
        expected.extend((x,y) for x in np.arange(x0,x1+.01,.05))
dist=tree.query(np.array(expected))[0]
def cell(p):
    mp=np.array(p)@R.T+np.array([tx,ty])
    index=np.floor((mp-origin[:2])/resolution).astype(int)
    return int(grid[index[1],index[0]]) if (0<=index[0]<grid.shape[1] and 0<=index[1]<grid.shape[0]) else -1
free_samples=[(0,0),(6,0)]+[(x,y) for y in (1.8,-2) for x in np.arange(0,6.3,.25)]
coverage=[cell(p) for p in free_samples]
trajectory=np.loadtxt(OUT/(run['run_id']+'-trajectory.csv'),delimiter=',',skiprows=1)
checks=dict(saved_from_slam=run['map_odom_nodes']==['slam_toolbox'] and run['manifest']['mode']=='mapping',
            resolution_5cm=resolution==.05,
            AB_and_two_corridors_free=all(v>=250 for v in coverage),
            wall_surface_p95_under_15cm=float(np.percentile(errors,95))<=.15,
            wall_surface_max_under_30cm=float(max(errors))<=.30,
            wall_completeness_p95_under_20cm=float(np.percentile(dist,95))<=.20,
            physical_closed_route=len(run['route'])==6 and np.linalg.norm(trajectory[-1,1:3])<=.4,
            no_clock_jump=run['clock_regressions']==0,
            updates_recorded=len(run['map_updates'])>10)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
checks={k:bool(v) for k,v in checks.items()}
stats=dict(checks=checks,verdict='PASS' if all(checks.values()) else 'FAIL',occupied_cells=len(xy),
           image_shape=list(grid.shape),resolution_m=resolution,origin=config['origin'],
           wall_surface_p95_m=float(np.percentile(errors,95)),wall_surface_max_m=float(max(errors)),
           expected_surface_p95_m=float(np.percentile(dist,95)),free_samples=len(coverage),
           free_sample_values=sorted(set(coverage)),map_updates=len(run['map_updates']),
           map_update_median_sim_s=float(np.median(np.diff(np.array(run['map_updates'])[:,0]))),
           loop_endpoint_m=trajectory[-1,1:3].tolist(),loop_distance_m=float(np.linalg.norm(trajectory[-1,1:3])),
           actual_path_length_m=float(np.linalg.norm(np.diff(trajectory[:,1:3],axis=0),axis=1).sum()),
           measured_vx_p95_m_s=float(np.percentile(abs(trajectory[:,4]),95)))
(OUT/'map-quality.json').write_text(json.dumps(stats,indent=2)+'\n')
meta=dict(schema=1,map_source='slam_toolbox /map saved with nav2_map_server/map_saver_cli; no geometry rasterization',
          map_yaml_sha256=sha(MAP.with_suffix('.yaml')),map_image_sha256=sha(MAP.parent/config['image']),
          scene_yaml='t1_mujoco/worlds/static_world.yaml',scene_yaml_sha256=manifest['scene_provenance']['layout']['sha256'],
          expanded_scene_sha256=manifest['scene_sha256'],upstream_sha=manifest['upstream_sha'],
          generated_code_sha=manifest['code_sha'],run_id=run['run_id'],created_utc=run['ended_utc'],
          sensor_mode='compensated',odom_mode='ideal_truth',lidar_config=manifest['lidar_config'],
          slam_params_sha256=sha(WS/'src/t1_navigation/config/slam.yaml'),
          T_world_map=dict(x=float(world_map_xy[0]),y=float(world_map_xy[1]),yaw=float(-a)),
          anchor=dict(map_to_odom_at_start=result['initial_map_odom'],odom_at_start=result['initial_odom'],
                      method='initial SLAM map->odom inverted, odom equals world truth horizontal projection (W06 contract)'),
          pose_graph=dict(path='docs/evidence/W09/raw/static_world.*',purpose='optional continuation/debug; localization only needs yaml/image',tracked=False),
          expected_localization=dict(initial_inputs=[[.2,-.15,.15],[-.2,.15,-.2],[.1,.1,.3]],
                                     convergence_sim_s=25,position_error_max_m=.25,yaw_error_max_rad=math.radians(10),scan_distance_p95_max_m=.15),
          limitations=['ideal truth odometry and truth attitude compensation','no drift/noise robustness claim','planar static scene only'])
MAP.with_suffix('.meta.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
fig,ax=plt.subplots(figsize=(9,8));ax.scatter(world[:,0],world[:,1],s=2,c='black',label='SLAM occupied cells')
ax.plot(trajectory[:,1],trajectory[:,2],color='tab:blue',label='Physical inspection route')
ax.scatter([0,6],[0,0],c=['green','red'],s=60);ax.text(0,.2,'A');ax.text(6,.2,'B')
ax.set_aspect('equal');ax.set(xlabel='world x (m)',ylabel='world y (m)',title='LiDAR SLAM map and actual T1 closed route')
ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'map-route.png',dpi=150)
print(json.dumps(stats,indent=2))
raise SystemExit(0 if stats['verdict']=='PASS' else 1)

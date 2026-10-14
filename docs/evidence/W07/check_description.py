"""C3: replay W05 traces, independently verify FK and measure body sweep."""
import csv
from datetime import datetime,timezone
import json, math, os, subprocess, time
from pathlib import Path
from urllib.parse import urlparse,unquote
import mujoco
import numpy as np
import torch
from model_geometry import UrdfFK, GeometryBounds, root_pose, origin
from t1_mujoco.runtime import load_task_config
from t1_mujoco.state_bridge import capture
from t1_bringup.description import robot_description
from t1_mujoco.recording import sha256
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W07';ASSETS=Path('/home/haoyang/robot_sim_sources/booster_assets')
started=datetime.now(timezone.utc).isoformat();wall=time.monotonic();torch.set_num_threads(1)
cfg=load_task_config(os.environ['T1_UPSTREAM'])
from t1_mujoco.runner import T1SimRunner
fk=UrdfFK(robot_description(ASSETS));r=T1SimRunner(cfg,debug=True)
m=r.mj_model;fresh=mujoco.MjData(m);bounds=GeometryBounds(m)
max_pos={m.body(i).name:0. for i in range(1,m.nbody)};max_rot=dict(max_pos)
mesh_errors={};extents=np.array([[np.inf,np.inf],[-np.inf,-np.inf]]);collision_extents=extents.copy();samples=0
# Mesh scale/zero-pose independent check: raw STL points vs compiled mesh points.
fresh.qpos[:]=0;fresh.qpos[3]=1;mujoco.mj_forward(m,fresh)
poses=fk.poses({n:0. for n in cfg.robot.joint_names},root_pose(fresh))
for link in fk.root.findall('link'):
    if link.get('name')=='base_link':continue
    for visual in link.findall('visual'):
        mesh=visual.find('geometry/mesh')
        if mesh is None:continue
        assert mesh.get('scale','1 1 1')=='1 1 1'
        path=Path(unquote(urlparse(mesh.get('filename')).path));blob=path.read_bytes()
        n=int.from_bytes(blob[80:84],'little');assert len(blob)==84+50*n
        raw=np.frombuffer(blob, dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]),offset=84,count=n)['vertices'].reshape(-1,3)
        t=poses[link.get('name')]@origin(visual.find('origin'));urdf=raw@t[:3,:3].T+t[:3,3]
        match=[i for i in range(m.ngeom) if m.geom_bodyid[i]==m.body(link.get('name')).id and m.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH and m.mesh(m.geom_dataid[i]).name.lower()==path.stem.lower()]
        assert len(match)==1,(path,match)
        i=match[0];mid=m.geom_dataid[i];a=m.mesh_vertadr[mid];n=m.mesh_vertnum[mid]
        compiled=m.mesh_vert[a:a+n]@fresh.geom_xmat[i].reshape(3,3).T+fresh.geom_xpos[i]
        mesh_errors[str(path.name)]=float(max(np.max(abs(urdf.min(axis=0)-compiled.min(axis=0))),np.max(abs(urdf.max(axis=0)-compiled.max(axis=0)))))

def check(runner):
    global samples,extents,collision_extents
    # Read a copy after mj_forward: active mj_step caches can lag by one substep.
    mujoco.mj_copyData(fresh,m,runner.mj_data);mujoco.mj_forward(m,fresh)
    snapshot=capture(runner)
    poses=fk.poses(dict(zip(snapshot.joint_names,snapshot.joint_position)),root_pose(fresh))
    for name in max_pos:
        t=poses[name];bid=m.body(name).id
        max_pos[name]=max(max_pos[name],float(np.linalg.norm(t[:3,3]-fresh.xpos[bid])))
        max_rot[name]=max(max_rot[name],float(np.max(abs(t[:3,:3]-fresh.xmat[bid].reshape(3,3)))))
    cp,ap=bounds.project(fresh)
    for points,limits in ((cp,collision_extents),(ap,extents)):
        limits[0]=np.minimum(limits[0],points.min(axis=0));limits[1]=np.maximum(limits[1],points.max(axis=0))
    samples+=1
# Offline zero/random configurations test transforms; no robot motion claimed.
rng=np.random.default_rng(707);offline_max=0.
for k in range(20):
    fresh.qpos[:]=0;fresh.qpos[3]=1
    if k:
        for j in range(1,m.njnt):fresh.qpos[m.jnt_qposadr[j]]=rng.uniform(*m.jnt_range[j])
    mujoco.mj_forward(m,fresh)
    poses=fk.poses({m.joint(j).name:fresh.qpos[m.jnt_qposadr[j]] for j in range(1,m.njnt)},root_pose(fresh))
    for name in max_pos:
        i=m.body(name).id;t=poses[name]
        offline_max=max(offline_max,float(np.linalg.norm(t[:3,3]-fresh.xpos[i])),float(np.max(abs(t[:3,:3]-fresh.xmat[i].reshape(3,3)))))
r.stop()
# Representative envelope corners from preserved W05 C3 raw trace, rep=0.
trace_paths=list((WS/'run_data').glob('*_dev_w05_c3/trace.csv'));assert len(trace_paths)==1
trace_path=trace_paths[0];selected={}
for row in csv.DictReader(trace_path.open()):
    trial=row['trial']
    if row['rep']=='0' and (trial.startswith('grid_ux0.4:') or trial.startswith('grid_ux1:')):
        if abs(float(trial.rsplit(',',1)[1])) in (0.,.5):selected.setdefault(trial,[]).append(row)
    elif row['rep']=='0' and trial.startswith('inplace:') and abs(float(trial.rsplit(',',1)[1])) in (.6,.8):
        selected.setdefault(trial,[]).append(row)
assert len(selected)==10,len(selected)
replays=[]
for trial,rows in selected.items():
    r=T1SimRunner(load_task_config(os.environ['T1_UPSTREAM']),debug=True)
    while r.state=='STARTING':assert r.step();check(r);r.finish_boundary()
    err=0.;start=time.monotonic()
    try:
        for row in rows:
            cmd=[float(row['u_pol_'+k]) for k in ('x','y','wz')]
            r.inputs.receive((cmd[0],cmd[1],0.,0.,0.,cmd[2]));assert r.step(),(trial,r.state)
            check(r);measured=r.measurements()
            for key in ('x_w','y_w','z_w','yaw_w','roll','pitch','v_meas_x','v_meas_y','v_meas_wz'):
                err=max(err,abs(measured[key]-float(row[key])))
            r.finish_boundary()
        replays.append(dict(trial=trial,steps=len(rows),sim_s=float(r.mj_data.time),wall_s=time.monotonic()-start,trace_max_error=err,final_state=r.state))
    finally:r.write_command((0.,0.,0.));r.stop()
    print(trial,'trace error',err,flush=True)
# Centimetre rounding outward + 0.05m geometric margin per side.
lo=np.floor((extents[0]-.05)*100)/100;hi=np.ceil((extents[1]+.05)*100)/100
polygon=[[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]
diameter=float(2*max(np.linalg.norm(v) for v in polygon))
checks=dict(joint_set_23={j[0] for j in fk.joints if j[1]=='revolute'}==set(cfg.robot.joint_names),
            fk_all_bodies=max(max(max_pos.values()),max(max_rot.values()),offline_max)<1e-5,
            mesh_units_and_visual_bounds=max(mesh_errors.values())<1e-5,
            w05_replay_exact=max(x['trace_max_error'] for x in replays)<1e-10,
            real_sweep_samples=samples>=9500,
            minimum_passage_margin=1.9-diameter>=.2)
result=dict(code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),cwd=str(WS),command='source scripts/env.sh; $T1_VENV/bin/python docs/evidence/W07/check_description.py',wall_s=time.monotonic()-wall,source_trace=dict(path=str(trace_path),sha256=sha256(trace_path),manifest=json.loads((trace_path.parent/'manifest.json').read_text())),samples=samples,offline_configs=20,offline_max_error=offline_max,body_position_errors_m=max_pos,body_rotation_matrix_errors=max_rot,mesh_visual_aabb_errors_m=mesh_errors,collision_extents_m=collision_extents.tolist(),all_geometry_extents_m=extents.tolist(),footprint=polygon,geometric_margin_per_side_m=.05,rotation_diameter_m=diameter,gate_width_m=1.9,gate_remaining_width_m=1.9-diameter,replays=replays,checks=checks,verdict='PASS' if all(checks.values()) else 'FAIL')
(OUT/'description-envelope.json').write_text(json.dumps(result,indent=2)+'\n')
print(checks,'extents',extents,'polygon',polygon,'diameter',diameter)
assert result['verdict']=='PASS'

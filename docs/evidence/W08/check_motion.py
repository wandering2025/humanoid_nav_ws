"""Paired physical trajectories, with/without sensor. No locomotion pose edits."""
from datetime import datetime,timezone
import json, os, subprocess, time, math
from pathlib import Path
import mujoco
import numpy as np
import torch
from t1_mujoco.runtime import load_task_config
from t1_mujoco.scenes import configure_scene
from t1_mujoco.command_map import CommandMap
from t1_mujoco.lidar import RayLidar, scan_transform
from t1_mujoco.state_bridge import capture
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W08';RAW=OUT/'raw';RAW.mkdir(exist_ok=True)
cfg=load_task_config(os.environ['T1_UPSTREAM'])
from t1_mujoco.runner import T1SimRunner
mapper=CommandMap.load(WS/'src/t1_mujoco/config/command_map.yaml');torch.set_num_threads(1)
started=datetime.now(timezone.utc).isoformat();t0=time.monotonic()
sequence=[('standing',3.,0.,0.),('forward',3.,.4,0.),('stop1',4.,0.,0.),('turn90',8.,0.,.6),('stop2',4.,0.,0.),('rotated_forward',4.,.4,0.),('stop3',4.,0.,0.)]
all_runs=[];base=None;turn_steps=None;plot_xy=[];plot_phase=[]
def wall_errors(points):
    return np.min(np.abs(points[:,:2,None]-np.array([[-2.,8.],[-5.,5.]])[None,:,:]).reshape(len(points),4),axis=1) if len(points) else np.array([])
for enabled in (False,True):
    torch.manual_seed(0);np.random.seed(0)
    cfg=load_task_config(os.environ['T1_UPSTREAM'])
    provenance=configure_scene(cfg,'calibration_world',Path(os.environ['T1_UPSTREAM']).parent/'booster_assets',WS/'run_data/cache/scenes',WS/'src/t1_mujoco/worlds')
    r=T1SimRunner(cfg,command_map=mapper);lidar=RayLidar(r) if enabled else None
    states=[];phases={};checks={};robot_hits=0;times=[];began=time.monotonic();tf_errors=[];minz=2.;contacts=0
    try:
        for phase,duration,vx,wz in sequence:
            phase_start=r.measurements();errs=[];rawerrs=[];known=[];scans=0;attitudes=[];point_count=0
            count = 0
            for _ in range(round(duration/.02)):
                if phase == 'turn90' and ((not enabled and math.degrees(r.measurements()['yaw_w']) >= 110) or (enabled and count == turn_steps)):
                    break
                count += 1
                r.inputs.receive((vx,0.,0.,0.,0.,wz));assert r.step(),(r.state,r.fault)
                states.append(np.r_[r.mj_data.qpos,r.mj_data.qvel]);meas=r.measurements();attitudes.append([meas['roll'],meas['pitch']]);minz=min(minz,meas['z_w'])
                for c in r.mj_data.contact:
                    if c.dist<=0 and any(r.mj_model.geom(int(i)).name.startswith('env_') for i in (c.geom1,c.geom2)):contacts+=1
                if lidar is not None and lidar.due(r.episode):
                    s=capture(r);f=lidar.sample(s);times.append(f.compute_ms);scans+=1;known.append(f.known_fraction)
                    assert f.healthy, (phase,f.known_fraction,f.blind_bins)
                    robot_hits+=int(lidar.robot_geoms[f.geom_ids].sum())
                    angles=-math.pi+np.arange(360)*2*math.pi/360;valid=np.isfinite(f.ranges)
                    local=np.c_[f.ranges[valid]*np.cos(angles[valid]),f.ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())]
                    world=f.origin+local@f.level_rotation.T
                    errs.extend(wall_errors(world));rawerrs.extend(wall_errors(f.projected_hits));point_count+=len(world)
                    plot_xy.extend(world[:,:2]);plot_phase.extend([phase]*len(world))
                    tf=scan_transform(s);p=tf.transform.translation
                    origin=np.array([*s.position[:2],0])+f.level_rotation@np.array([p.x,p.y,p.z])
                    tf_errors.append(float(np.linalg.norm(origin-f.origin)))
                r.finish_boundary()
            if phase == 'turn90' and not enabled:turn_steps=count
            duration = count*.02
            end=r.measurements();att=np.array(attitudes)
            phases[phase]=dict(sim_s=duration,scans=scans,point_count=point_count,
                displacement_m=[end['x_w']-phase_start['x_w'],end['y_w']-phase_start['y_w']],yaw_change_deg=math.degrees(math.remainder(end['yaw_w']-phase_start['yaw_w'],math.tau)),end_yaw_deg=math.degrees(end['yaw_w']),
                roll_pitch_span_deg=np.rad2deg(np.ptp(att,axis=0)).tolist(),roll_pitch_abs_max_deg=np.rad2deg(np.max(abs(att),axis=0)).tolist(),
                wall_p95_m=float(np.percentile(errs,95)) if errs else None,wall_max_m=float(max(errs)) if errs else None,
                raw_wall_max_m=float(max(rawerrs)) if rawerrs else None,known_min=min(known) if known else None)
        values=np.array(states)
        if base is None:base=values
        diff=float(np.max(abs(values-base)))
        checks=dict(physical_state_unchanged=diff<1e-10,no_fall=minz>.6 and r.state=='READY',no_environment_contact_at_policy_samples=contacts==0,
                    actual_turn_near_90=abs(phases['stop2']['end_yaw_deg']-90)<20,
                    forward_displacement=phases['forward']['displacement_m'][0]>.7,
                    rotated_forward_displacement=phases['rotated_forward']['displacement_m'][1]>.8)
        if enabled:
            checks.update(wall_p95_under_8cm=all(v['wall_p95_m']<=.08 for v in phases.values()),wall_max_under_12cm=all(v['wall_max_m']<=.12 for v in phases.values()),
                raw_wall_under_1nm=all(v['raw_wall_max_m']<1e-9 for v in phases.values()),robot_hits_zero=robot_hits==0,
                sample_p95_under_50ms=np.percentile(times,95)<50,tf_origin_consistent=max(tf_errors)<1e-12)
            # Exercise actual policy command gate: remain dynamically standing.
            r.sensor_required=True;r.sensor_healthy=False;r.inputs.receive((.4,0.,0.,0.,0.,0.));assert r.step()
            checks['sensor_health_blocks_command']=r.u_pol==(0.,0.,0.) and r.cmd_status=='BLOCKED_BY_STATE'
            r.sensor_healthy=True;r.inputs.receive((.4,0.,0.,0.,0.,0.));assert r.step()
            checks['fresh_healthy_command_resumes']=r.u_pol[0]>0
            r.reset_episode();checks['reset_requires_new_scan']=not r.sensor_healthy and r.u_pol==(0.,0.,0.)
        all_runs.append(dict(enabled=enabled,phases=phases,checks={k:bool(v) for k,v in checks.items()},qpos_qvel_max_error=diff,
            min_trunk_z_m=float(minz),wall_s=time.monotonic()-began,sim_s=sum(x['sim_s'] for x in phases.values()),robot_hits=robot_hits,
            scan_compute_ms=dict(p50=float(np.median(times)),p95=float(np.percentile(times,95)),max=float(max(times))) if times else None,provenance=provenance))
        print(enabled,all_runs[-1]['checks'],all_runs[-1]['scan_compute_ms'],flush=True)
    finally:r.write_command((0.,0.,0.));r.stop()
np.savez_compressed(RAW/'wall-points.npz',xy=np.array(plot_xy),phase=np.array(plot_phase))
result=dict(code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),cwd=str(WS),command='$T1_VENV/bin/python docs/evidence/W08/check_motion.py',wall_s=time.monotonic()-t0,sequence=sequence,runs=all_runs,verdict='PASS' if all(all(x['checks'].values()) for x in all_runs) else 'FAIL')
(OUT/'motion.json').write_text(json.dumps(result,indent=2)+'\n');assert result['verdict']=='PASS'

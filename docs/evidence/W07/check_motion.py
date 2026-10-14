"""Physical forward/turn/stop in each scene vs unchanged upstream baseline."""
from datetime import datetime,timezone
import json, os, subprocess, time
from pathlib import Path
from unittest.mock import patch
import mujoco
import numpy as np
import torch
from t1_mujoco.runtime import load_task_config
from t1_mujoco.command_map import CommandMap
from t1_mujoco.scenes import SCENES,configure_scene
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W07'
cfg=load_task_config(os.environ['T1_UPSTREAM'])
from t1_mujoco.runner import T1SimRunner
mapper=CommandMap.load(WS/'src/t1_mujoco/config/command_map.yaml');torch.set_num_threads(1)
started=datetime.now(timezone.utc).isoformat();wall=time.monotonic()
# Direct program inputs in physical units through W05 guard; no pose edits.
sequence=[(3.,(0.,0.)),(2.,(.4,0.)),(2.,(.4,.3)),(4.,(0.,0.)),(2.,(0.,.6)),(4.,(0.,0.))]
original_step=mujoco.mj_step;baseline=None;results=[]
for name in ('upstream',*SCENES):
    cfg=load_task_config(os.environ['T1_UPSTREAM'])
    provenance=configure_scene(cfg,'' if name=='upstream' else name,Path(os.environ['T1_UPSTREAM']).parent/'booster_assets',WS/'run_data/cache/scenes',WS/'src/t1_mujoco/worlds')
    r=T1SimRunner(cfg,command_map=mapper);samples=[];contacts=[];zmin=1.;t0=time.monotonic();substeps=0
    def observe(model,data):
        global substeps
        original_step(model,data);substeps+=1
        for c in data.contact:
            if c.dist>0:continue
            ids=(int(c.geom1),int(c.geom2))
            if any(model.geom(i).name.startswith('env_') for i in ids):
                contacts.append(dict(t=float(data.time),geoms=[model.geom(i).name for i in ids],dist=float(c.dist)))
    try:
        with patch('mujoco.mj_step',side_effect=observe):
            segments=[]
            for duration,(vx,wz) in sequence:
                segment=[]
                for _ in range(round(duration/.02)):
                    r.inputs.receive((vx,0.,0.,0.,0.,wz));assert r.step(),(name,r.state,r.fault)
                    samples.append(np.r_[r.mj_data.qpos,r.mj_data.qvel]);segment.append(r.measurements());zmin=min(zmin,r.mj_data.qpos[2]);r.finish_boundary()
                segments.append(segment)
        values=np.array(samples)
        if baseline is None:baseline=values
        error=float(np.max(abs(values-baseline)))
        forward=float(np.mean([s['v_meas_x'] for s in segments[1][50:]]))
        turn=float(np.mean([s['v_meas_wz'] for s in segments[2][50:]]))
        spin=float(np.mean([s['v_meas_wz'] for s in segments[4][50:]]))
        final=segments[-1][-100:]
        speed=float(max(np.hypot(s['v_meas_x'],s['v_meas_y']) for s in final))
        checks=dict(full_state_same_as_upstream=error<1e-10,forward=forward>.2,walking_turn=turn>.15,inplace_turn=spin>.4,no_env_contacts=len(contacts)==0,no_fall=r.state=='READY' and zmin>.6,stopped=speed<.05)
        checks={key:bool(value) for key,value in checks.items()}
        results.append(dict(scene=name,checks=checks,sim_s=float(r.mj_data.time),wall_s=time.monotonic()-t0,physics_substeps=substeps,qpos_qvel_max_error=error,forward_m_s=forward,walking_turn_rad_s=turn,inplace_turn_rad_s=spin,final_speed_max_m_s=speed,min_z_m=float(zmin),environment_contacts=contacts,provenance=provenance))
        print(name,checks,flush=True)
        (WS/'run_data'/f'w07-{name}-states.npy').write_bytes(values.tobytes())
    finally:r.write_command((0.,0.,0.));r.stop()
result=dict(code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),wall_s=time.monotonic()-wall,cwd=str(WS),command='source scripts/env.sh; $T1_VENV/bin/python docs/evidence/W07/check_motion.py',sequence=sequence,runs=results,verdict='PASS' if all(all(r['checks'].values()) for r in results) else 'FAIL')
(OUT/'scene-motion.json').write_text(json.dumps(result,indent=2)+'\n');assert result['verdict']=='PASS'

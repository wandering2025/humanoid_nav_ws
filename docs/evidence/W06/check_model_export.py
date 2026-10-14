"""Owner-thread live state export checks against real named MuJoCo addresses.

Uses the physical runner/torque policy, no base-pose edits. mj_forward below
operates only on a copy for an independent fresh mjOBJ_XBODY velocity check.
"""
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import mujoco
import numpy as np
import torch
from t1_mujoco.runtime import load_task_config
from t1_mujoco.command_map import CommandMap
from t1_mujoco.state_bridge import capture, messages
cfg=load_task_config(os.environ['T1_UPSTREAM'])
from t1_mujoco.runner import T1SimRunner
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W06'
torch.set_num_threads(1)
r=T1SimRunner(cfg,command_map=CommandMap.load(WS/'src/t1_mujoco/config/command_map.yaml'))
copy=mujoco.MjData(r.mj_model)
started=datetime.now(timezone.utc).isoformat();wall0=time.monotonic()
errors=dict(position=0.,velocity=0.,effort=0.,body_linear=0.,world_angular_z=0.,truth_orientation=0.,v_meas=0.)
counts=dict(standing=0,moving=0)
positions=[]
try:
    for k in range(400):
        moving=k>=150
        r.inputs.receive((.4 if moving else 0.,0.,0.,0.,0.,0.))
        assert r.step(),(r.state,r.fault)
        s=capture(r);odom,tf,joints,truth=messages(s)
        mujoco.mj_copyData(copy,r.mj_model,r.mj_data)
        mujoco.mj_forward(r.mj_model,copy)
        velocity_local=np.empty(6);velocity_world=np.empty(6)
        for local, dest in [(1,velocity_local),(0,velocity_world)]:
            mujoco.mj_objectVelocity(r.mj_model,copy,mujoco.mjtObj.mjOBJ_XBODY,r.mj_model.body('trunk').id,dest,local)
        actual=[xyz for xyz in (truth.twist.twist.linear.x,truth.twist.twist.linear.y,truth.twist.twist.linear.z)]
        errors['body_linear']=max(errors['body_linear'],float(np.max(np.abs(np.asarray(actual)-velocity_local[3:]))))
        errors['world_angular_z']=max(errors['world_angular_z'],abs(odom.twist.twist.angular.z-velocity_world[2]))
        for i,name in enumerate(joints.name):
            joint=r.mj_model.joint(name)
            qa=int(r.mj_model.jnt_qposadr[joint.id]);va=int(r.mj_model.jnt_dofadr[joint.id])
            for key,exported,expected in [('position',joints.position[i],r.mj_data.qpos[qa]),('velocity',joints.velocity[i],r.mj_data.qvel[va]),('effort',joints.effort[i],r.mj_data.qfrc_actuator[va])]:
                errors[key]=max(errors[key],abs(float(exported)-float(expected)))
        errors['truth_orientation']=max(errors['truth_orientation'],float(np.max(np.abs(np.array([truth.pose.pose.orientation.w,truth.pose.pose.orientation.x,truth.pose.pose.orientation.y,truth.pose.pose.orientation.z])-np.array(s.quaternion_wxyz)))))
        measured=r.measurements()
        for actual,key in [(odom.twist.twist.linear.x,'v_meas_x'),(odom.twist.twist.linear.y,'v_meas_y'),(odom.twist.twist.angular.z,'v_meas_wz')]:
            errors['v_meas']=max(errors['v_meas'],abs(actual-measured[key]))
        counts['moving' if moving else 'standing']+=1
        positions.append([s.position[0],s.position[1]])
        r.finish_boundary()
    rejected=[]
    def other_thread():
        try:
            capture(r)
        except RuntimeError as exc:
            rejected.append(str(exc))
    worker=threading.Thread(target=other_thread);worker.start();worker.join()
    checks=dict(fields_exact=max(errors.values())<1e-12,joints_23=len(joints.name)==23,
                real_motion=float(np.linalg.norm(np.array(positions[-1])-positions[149]))>1.,
                other_thread_rejected=len(rejected)==1,all_steps_ready=r.state=='READY')
    result=dict(started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),wall_s=time.monotonic()-wall0,
                code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                command='source scripts/env.sh; python docs/evidence/W06/check_model_export.py',cwd=str(WS),
                sim_duration_s=float(r.mj_data.time),samples=counts,joint_names=list(joints.name),
                units=dict(position='rad',velocity='rad/s',effort='N*m'),max_errors=errors,
                checks=checks,verdict='PASS' if all(checks.values()) else 'FAIL')
    (OUT/'model-export.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result)
    assert all(checks.values())
finally:
    r.write_command((0.,0.,0.));r.stop()

"""Frozen upstream numerical reference; bypass only viewer/sleep/stdin I/O."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from datetime import datetime, timezone
import numpy as np
from t1_mujoco.runtime import load_task_config


def run():
    cfg = load_task_config(os.environ['T1_UPSTREAM'])
    from booster_deploy.controllers.mujoco_controller import MujocoController
    import mujoco
    import torch
    torch.set_num_threads(1)
    ctrl = MujocoController(cfg)
    # I/O only: official update_state/policy_step/ctrl_step remain unchanged.
    ctrl.update_vel_command = lambda: None
    ctrl.update_state()
    ctrl.start()
    rows = []
    started = datetime.now(timezone.utc).isoformat()
    wall = time.monotonic()
    for k in range(750):  # 5 s standing + 10 s u_pol.x=0.5
        ctrl.vel_command.lin_vel_x = 0.0 if k < 250 else 0.5
        ctrl.update_state()
        target = ctrl.policy_step()
        if not ctrl.is_running:
            raise RuntimeError('upstream fell')
        ctrl.ctrl_step(target)
        assert np.isfinite(ctrl.mj_data.qpos).all()
        rows.append(np.r_[ctrl.mj_data.time, ctrl.mj_data.qpos.copy(), ctrl.mj_data.qvel.copy()])
    duration = time.monotonic() - wall
    Path('run_data/w04_baseline').mkdir(parents=True, exist_ok=True)
    np.save('run_data/w04_baseline/trajectory.npy', rows)
    mapping = []
    m = ctrl.mj_model
    for name in cfg.robot.joint_names:
        j = m.joint(name).id
        a = np.flatnonzero(m.actuator_trnid[:, 0] == j)
        mapping.append(dict(name=name, joint_id=j, qpos=int(m.jnt_qposadr[j]), qvel=int(m.jnt_dofadr[j]), actuator=a.tolist()))
    result = dict(started_utc=started, ended_utc=datetime.now(timezone.utc).isoformat(),
                  code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                  upstream_sha=subprocess.check_output(['git','-C',os.environ['T1_UPSTREAM'],'rev-parse','HEAD'],text=True).strip(),
                  policy_dt=cfg.policy_dt, physics_dt=cfg.mujoco.physics_dt, decimation=cfg.mujoco.decimation,
                  initial_pos=cfg.mujoco.init_pos, initial_quat=cfg.mujoco.init_quat,
                  default_joint_pos=cfg.robot.default_joint_pos, kp=cfg.robot.joint_stiffness,
                  kd=cfg.robot.joint_damping, effort_limit=cfg.robot.effort_limit,
                  joint_mapping=mapping, policy_joint_names=cfg.policy.policy_joint_names,
                  sim_duration=float(ctrl.mj_data.time), wall_duration=duration, rtf=15/duration,
                  standing_end=rows[249][1:4].tolist(), walking_end=rows[-1][1:4].tolist(),
                  walking_displacement=(rows[-1][1:4]-rows[249][1:4]).tolist(),
                  min_height=float(np.min(np.array(rows)[:,3])),
                  trajectory_sha256=hashlib.sha256(np.array(rows).tobytes()).hexdigest(), exit_code=0)
    Path('docs/evidence/W04/c1-baseline.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('sim_duration','wall_duration','rtf','walking_displacement','min_height')}))

if __name__ == '__main__':
    run()

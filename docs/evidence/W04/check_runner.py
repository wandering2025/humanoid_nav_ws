"""Non-ROS physics comparison and fault/reset checks against frozen policy."""
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from datetime import datetime, timezone
import numpy as np
import torch
from t1_mujoco.runtime import load_task_config
cfg = load_task_config(os.environ['T1_UPSTREAM'])
from t1_mujoco.runner import T1SimRunner
from t1_mujoco.model_contract import joint_map, ModelIncompatible
from t1_mujoco.control import Request

started = datetime.now(timezone.utc).isoformat()
wall = time.monotonic()
torch.set_num_threads(1)
r = T1SimRunner(cfg, debug=True, warmup=0.)
r.finish_boundary()
rows=[]
for k in range(750):
    r.inputs.receive((0. if k<250 else .5,0,0,0,0,0))
    assert r.step()
    rows.append(np.r_[r.mj_data.time,r.mj_data.qpos.copy(),r.mj_data.qvel.copy()])
    r.finish_boundary()
reference=np.load('run_data/w04_baseline/trajectory.npy')
error=float(np.max(np.abs(reference-np.array(rows))))
assert error < 1e-10, error
assert r.policy.obs_history is not None
old_time=float(r.mj_data.time)
r.reset_episode()
assert r.policy.obs_history is None and torch.count_nonzero(r.policy.last_action)==0
assert torch.equal(r.policy.filtered_dof_target,r.robot.default_joint_pos)
assert r._step_count==0 and r._elapsed_s==0 and r.episode==1
assert r.inputs.command is None and r.u_pol==(0.,0.,0.) and r.mj_data.time > old_time
checks=['trajectory_matches_upstream','reset_history_action_filter_counters_commands_monotonic_time']

# Fault injection only in this test process: no production injection parameters.
for kind in ('nan_action','policy_exception','nan_state','fallen'):
    r=T1SimRunner(cfg)
    if kind=='nan_action':
        r.policy.inference=lambda: torch.full((23,),float('nan'))
    elif kind=='policy_exception':
        def fail(): raise RuntimeError('test fault')
        r.policy.inference=fail
    elif kind=='nan_state':
        r.mj_data.qvel[0]=float('nan')
    else:
        r.mj_data.qpos[3:7]=[0.,1.,0.,0.]
    t=float(r.mj_data.time)
    assert not r.step()
    assert r.state==('FALLEN' if kind=='fallen' else 'FAULT')
    assert r.u_pol==(0.,0.,0.) and r.mj_data.time==t and not r.step()
    checks.append(kind)

r=T1SimRunner(cfg)
errors=[]
def wrong_thread():
    try: r.step()
    except RuntimeError as exc: errors.append(str(exc))
th=threading.Thread(target=wrong_thread);th.start();th.join()
assert errors
checks.append('single_owner_assertion')
req=Request('pause',True,deadline=time.monotonic()-1)
r.requests.queue.put(req);r.handle_requests('sim_only')
assert not req.result[0] and r.state=='STARTING'
checks.append('expired_request_never_applied')
req=Request('reset');r.requests.queue.put(req);r.handle_requests('mapping')
assert not req.result[0] and r.episode==0
checks.append('non_sim_only_reset_rejected')
for corrupt in ('order','gear'):
    r=T1SimRunner(cfg)
    if corrupt=='order': names=list(reversed(cfg.robot.joint_names))
    else:
        names=cfg.robot.joint_names;r.mj_model.actuator_gear[0,0]=2
    try: joint_map(r.mj_model,names)
    except ModelIncompatible: pass
    else: raise AssertionError('incompatible model accepted')
    checks.append('model_rejects_'+corrupt)
result=dict(started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),
            code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            sim_duration=15.,wall_duration=time.monotonic()-wall,max_trajectory_error=error,checks=checks,exit_code=0)
Path('docs/evidence/W04/check-runner.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))

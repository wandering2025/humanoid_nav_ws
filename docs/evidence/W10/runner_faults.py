"""F12/F13 finite state/policy hooks, isolated from ROS. Not locomotion evidence."""
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import time
import numpy as np
import torch
import mujoco
from t1_mujoco.runtime import load_task_config
from t1_mujoco.command_map import CommandMap

WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W10'
cfg=load_task_config(os.environ['T1_UPSTREAM'])
from t1_mujoco.runner import T1SimRunner

torch.set_num_threads(1)
mapper=CommandMap.load(WS/'src/t1_mujoco/config/command_map.yaml')
results=[]
for case in ('height','tilt','policy_exception','nonfinite_action','nonfinite_state'):
    runner=T1SimRunner(cfg,command_map=mapper)
    trace=[]
    for i in range(350):
        if runner.state=='READY':runner.inputs.receive((.5,0,0,0,0,0))
        assert runner.step()
        if i%10==0:trace.append(dict(sim=float(runner.mj_data.time),wall=time.monotonic(),state=runner.state,u=runner.u_pol,**runner.measurements()))
        runner.finish_boundary()
    injected=float(runner.mj_data.time)
    if case=='height':runner.mj_data.qpos[2]=.4
    if case=='tilt':
        runner.mj_data.qpos[3:7]=[math.cos(math.radians(70)/2),math.sin(math.radians(70)/2),0,0]
        mujoco.mj_forward(runner.mj_model,runner.mj_data)
    if case=='policy_exception':
        def failure():raise RuntimeError('W10 controlled policy exception')
        runner.policy_step=failure
    if case=='nonfinite_action':runner.policy_step=lambda:torch.full((23,),float('nan'))
    if case=='nonfinite_state':runner.mj_data.qvel[0]=float('nan')
    advanced=runner.step()
    expected='FALLEN' if case in ('height','tilt') else 'FAULT'
    checks=dict(expected_state=runner.state==expected,zero=runner.u_pol==(0.,0.,0.),
                no_physics_advance=not advanced and float(runner.mj_data.time)==injected,terminal_failure=runner.had_failure)
    runner.inputs.receive((.5,0,0,0,0,0))
    checks['no_automatic_recovery']=not runner.step() and runner.state==expected
    result=dict(case=case,injection='isolated runner controlled state/policy injection; no production hook',
                injection_sim=injected,detected_sim=float(runner.mj_data.time),state=runner.state,fault=runner.fault,
                checks=checks,verdict='PASS' if all(checks.values()) else 'FAIL',trace=trace)
    results.append(result);runner.stop();print(case,result['verdict'],flush=True)
report=dict(tested_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            completed_utc=datetime.now(timezone.utc).isoformat(),cases=results,
            verdict='PASS' if all(r['verdict']=='PASS' for r in results) else 'FAIL')
(OUT/'runner-faults.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
raise SystemExit(0 if report['verdict']=='PASS' else 1)

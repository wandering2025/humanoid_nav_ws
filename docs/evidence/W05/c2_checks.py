"""W05 C2: separate adapter/frame errors from the policy's own low-command behaviour.

1. upstream_vs_adapter: frozen upstream MujocoController (I/O bypass only, as
   W04 baseline.py) vs T1SimRunner on identical command schedules near the
   threshold. Identical trajectories => the dead zone is not adapter-induced.
2. frame: C1 u=0.5 trials at initial headings 0/90/180 deg. World displacement
   must rotate with the heading while body-frame metrics stay equal; integrated
   body-frame v_meas must reproduce the world displacement.
3. policy_probe: read-only inference on a COPY of the standing obs history with
   only the command channels changed; weights, PD and observation layout are
   untouched. Measures how strongly actions react to each command.

usage: python c2_checks.py <c1 run dir> <out json>
"""
import copy
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import torch

from t1_mujoco.runtime import load_task_config

torch.set_num_threads(1)
cfg = load_task_config(os.environ['T1_UPSTREAM'])
from booster_deploy.controllers.mujoco_controller import MujocoController  # noqa: E402
from t1_mujoco.runner import T1SimRunner  # noqa: E402

c1_dir, out_path = Path(sys.argv[1]), Path(sys.argv[2])
started, wall0 = datetime.now(timezone.utc).isoformat(), time.monotonic()
result = dict(started_utc=started, code_sha=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
              dirty=subprocess.check_output(['git', 'status', '--short'], text=True).splitlines(),
              c1_run=str(c1_dir))

# 1. upstream vs adapter -------------------------------------------------------
STAND, MOVE = 150, 500  # 3 s zero, 10 s command (policy steps)


def upstream(u):
    ctrl = MujocoController(copy.deepcopy(cfg))
    ctrl.update_vel_command = lambda: None
    ctrl.update_state()
    ctrl.start()
    rows = []
    for k in range(STAND + MOVE):
        ctrl.vel_command.lin_vel_x, ctrl.vel_command.lin_vel_y, ctrl.vel_command.ang_vel_yaw = (0, 0, 0) if k < STAND else u
        ctrl.update_state()
        target = ctrl.policy_step()
        assert ctrl.is_running
        ctrl.ctrl_step(target)
        rows.append(np.r_[ctrl.mj_data.qpos, ctrl.mj_data.qvel])
    return np.array(rows)


def adapter(u):
    r = T1SimRunner(copy.deepcopy(cfg), debug=True, warmup=0.0)
    r.finish_boundary()
    rows = []
    for k in range(STAND + MOVE):
        r.inputs.receive(((0, 0, 0) if k < STAND else u)[:2] + (0, 0, 0) + (((0, 0, 0) if k < STAND else u)[2],))
        assert r.step()
        r.finish_boundary()
        rows.append(np.r_[r.mj_data.qpos, r.mj_data.qvel])
    return np.array(rows)


comparison = []
for u in ((0.3, 0.0, 0.0), (0.35, 0.0, 0.0), (0.375, 0.0, 0.0), (0.0, 0.0, 0.3), (0.0, 0.0, -0.3), (0.0, 0.3, 0.0)):
    a, b = upstream(u), adapter(u)
    comparison.append(dict(u_pol=u, max_abs_error=float(np.max(np.abs(a - b))),
                           upstream_disp_xy=(a[-1, :2] - a[STAND - 1, :2]).tolist(),
                           adapter_disp_xy=(b[-1, :2] - b[STAND - 1, :2]).tolist()))
    print('upstream_vs_adapter', comparison[-1], flush=True)
result['upstream_vs_adapter'] = comparison

# 2. frame check from C1 trace ----------------------------------------------------
frames = []
trace = [r for r in csv.DictReader(open(c1_dir / 'trace.csv')) if r['trial'] == 'vx:+0.500,+0.000,+0.000']
for rep in ('0', '1', '2'):
    rows = [r for r in trace if r['rep'] == rep]
    seg = [r for r in rows if r['segment'] == '1']
    before = rows[rows.index(seg[0]) - 1]
    f = lambda r, k: float(r[k])
    dx, dy = f(seg[-1], 'x_w') - f(before, 'x_w'), f(seg[-1], 'y_w') - f(before, 'y_w')
    yaw0 = f(before, 'yaw_w')
    # Integrate body-frame v_meas rotated by the instantaneous yaw (qvel-based).
    ix = iy = 0.0
    for r in seg:
        c, s = math.cos(f(r, 'yaw_w')), math.sin(f(r, 'yaw_w'))
        ix += (c * f(r, 'v_meas_x') - s * f(r, 'v_meas_y')) * 0.02
        iy += (s * f(r, 'v_meas_x') + c * f(r, 'v_meas_y')) * 0.02
    fwd = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    lat = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    frames.append(dict(heading_deg=[0, 90, 180][int(rep)], initial_yaw_deg=math.degrees(yaw0),
                       world_dx=dx, world_dy=dy, body_forward=fwd, body_lateral=lat,
                       integrated_vmeas_world=[ix, iy], integration_error_m=math.hypot(ix - dx, iy - dy)))
    print('frame', frames[-1], flush=True)
result['frame_check'] = frames

# 3. policy probe ---------------------------------------------------------------
r = T1SimRunner(copy.deepcopy(cfg), debug=True, warmup=0.0)
r.finish_boundary()
for _ in range(150):  # 3 s standing to fill the 10-frame history
    r.inputs.receive((0, 0, 0, 0, 0, 0))
    assert r.step()
history = r.policy.obs_history.clone()
model = r.policy._model


def action(cmd, frames='all'):
    h = history.clone()
    rows = slice(None) if frames == 'all' else slice(-1, None)
    h[rows, 6:9] = torch.tensor(cmd, dtype=torch.float32)  # command channels only
    with torch.no_grad():
        return model(h.flatten()).reshape(-1)


base = action((0, 0, 0))
probe = []
for axis, name in ((0, 'vx'), (1, 'vy'), (2, 'wz')):
    for v in (0.05, 0.1, 0.2, 0.3, 0.35, 0.375, 0.4, 0.5, 0.75, 1.0, -0.3, -0.5):
        cmd = [0.0, 0.0, 0.0]
        cmd[axis] = v
        probe.append(dict(axis=name, u=v, d_action_all_frames=float(torch.linalg.vector_norm(action(cmd) - base)),
                          d_action_last_frame=float(torch.linalg.vector_norm(action(cmd, 'last') - base))))
result['policy_probe'] = dict(state='standing after 3 s zero command; obs_history copied; command channels 6:9 replaced',
                              base_action_norm=float(torch.linalg.vector_norm(base)), rows=probe)
for p in probe:
    print('probe', p, flush=True)
result.update(ended_utc=datetime.now(timezone.utc).isoformat(), wall_s=time.monotonic() - wall0, exit_code=0)
out_path.write_text(json.dumps(result, indent=2) + '\n')

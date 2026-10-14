"""Recheck frozen MuJoCo free-joint velocity semantics; no physical run."""
import json
from pathlib import Path
import mujoco
import numpy as np
from t1_mujoco.runtime import load_task_config
import os
cfg = load_task_config(os.environ['T1_UPSTREAM'])
from booster_assets import BOOSTER_ASSETS_DIR
model = mujoco.MjModel.from_xml_path(cfg.robot.mjcf_path.format(BOOSTER_ASSETS_DIR=BOOSTER_ASSETS_DIR))
data = mujoco.MjData(model)
quat = np.empty(4)
mujoco.mju_euler2Quat(quat, np.array([0.3, -0.2, np.pi/2]), 'xyz')
data.qpos[:7] = [1, 2, .7, *quat]
data.qvel[:6] = [.2, .4, -.1, .3, -.2, .5]
mujoco.mj_forward(model, data)
rotation = data.xmat[model.body('trunk').id].reshape(3,3)
values = {}
for name, objtype, local in [('xbody_world', mujoco.mjtObj.mjOBJ_XBODY, 0),
                             ('xbody_local', mujoco.mjtObj.mjOBJ_XBODY, 1),
                             ('com_world', mujoco.mjtObj.mjOBJ_BODY, 0)]:
    res = np.empty(6)
    mujoco.mj_objectVelocity(model, data, objtype, model.body('trunk').id, res, local)
    values[name] = res.tolist()
checks = dict(world_linear=np.allclose(values['xbody_world'][3:], data.qvel[:3]),
              world_angular=np.allclose(values['xbody_world'][:3], rotation @ data.qvel[3:6]),
              local_linear=np.allclose(values['xbody_local'][3:], rotation.T @ data.qvel[:3]),
              local_angular=np.allclose(values['xbody_local'][:3], data.qvel[3:6]),
              com_differs=not np.allclose(values['com_world'][3:], data.qvel[:3]))
result = dict(mujoco_version=mujoco.__version__, setup='tilted trunk; mixed translation/rotation; mj_forward only',
              qpos=data.qpos[:7].tolist(), qvel=data.qvel[:6].tolist(), results=values,
              checks={k: bool(v) for k,v in checks.items()})
Path('docs/evidence/W06/velocity-probe.json').write_text(json.dumps(result, indent=2)+'\n')
print(result['checks'])
assert all(checks.values())

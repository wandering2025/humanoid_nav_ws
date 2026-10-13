"""W02 read-only interface probe.

Checks the upstream contract facts that docs/architecture/*.md depend on:
task config timing, joint/actuator ordering, policy I/O size, free-joint
velocity frames, sensor semantics and the MuJoCo API surface used later.

Does not step physics, does not write to upstream repos (bytecode disabled).
Run from $WS after sourcing ROS and the venv:
    python -B docs/evidence/W02/probe_interfaces.py docs/evidence/W02/c1-probe.json
"""
from __future__ import annotations

import datetime
import inspect
import json
import os
import pkgutil
import sys

sys.dont_write_bytecode = True
UPSTREAM = "/home/haoyang/robot_sim_sources/booster_deploy"
sys.path.insert(0, UPSTREAM)

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

out: dict = {"started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat()}
checks: dict[str, bool] = {}


def check(name: str, ok: bool) -> None:
    checks[name] = bool(ok)


# --- task config, same registry walk as scripts/deploy.py -------------------
import tasks as tasks_pkg  # noqa: E402

for mod in pkgutil.walk_packages(tasks_pkg.__path__, prefix="tasks."):
    __import__(mod.name)
from booster_deploy.utils.registry import get_task  # noqa: E402
from booster_deploy.controllers.mujoco_controller import MujocoController  # noqa: E402

cfg = get_task("t1_walk")
rc = cfg.robot
out["task"] = {
    "policy_dt": cfg.policy_dt,
    "decimation": cfg.mujoco.decimation,
    "physics_dt": cfg.mujoco.physics_dt,
    "init_pos": list(cfg.mujoco.init_pos),
    "init_quat": list(cfg.mujoco.init_quat),
    "vel_command": {
        "vx_max": cfg.vel_command.vx_max,
        "vx_forward_max": cfg.vel_command.vx_forward_max,
        "vx_backward_max": cfg.vel_command.vx_backward_max,
        "vy_max": cfg.vel_command.vy_max,
        "vyaw_max": cfg.vel_command.vyaw_max,
    },
    "policy_cls": f"{cfg.policy.constructor.__module__}.{cfg.policy.constructor.__qualname__}",
    "checkpoint_path": cfg.policy.checkpoint_path,
    "history_len": cfg.policy.actor_obs_history_length,
    "history_init": cfg.policy.history_init,
    "action_scale": cfg.policy.action_scale,
    "action_filter": cfg.policy.action_filter,
    "clip_action": cfg.policy.clip_action,
    "clip_observation": cfg.policy.clip_observation,
    "obs_dof_vel_scale": cfg.policy.obs_dof_vel_scale,
    "enable_safety_fallback": cfg.policy.enable_safety_fallback,
    "n_policy_joints": len(cfg.policy.policy_joint_names),
    "joints_not_in_policy": [j for j in rc.joint_names if j not in cfg.policy.policy_joint_names],
    "joint_names": list(rc.joint_names),
    "kp": list(rc.joint_stiffness),
    "kd": list(rc.joint_damping),
    "effort_limit": list(rc.effort_limit),
    "default_joint_pos": list(rc.default_joint_pos),
    "mjcf_path": rc.mjcf_path,
}
check("physics_dt == policy_dt/decimation", abs(cfg.mujoco.physics_dt * cfg.mujoco.decimation - cfg.policy_dt) < 1e-12)
out["run_signatures"] = {
    name: str(inspect.signature(getattr(MujocoController, name)))
    for name in ("start", "update_state", "policy_step", "ctrl_step", "update_vel_command", "run", "stop")
}

# --- model structure ----------------------------------------------------------
from booster_assets import BOOSTER_ASSETS_DIR  # noqa: E402

mjcf = rc.mjcf_path.replace("{BOOSTER_ASSETS_DIR}", str(BOOSTER_ASSETS_DIR))
m = mujoco.MjModel.from_xml_path(mjcf)
J = mujoco.mjtObj.mjOBJ_JOINT
B = mujoco.mjtObj.mjOBJ_BODY
joints = []
for j in range(m.njnt):
    joints.append({
        "id": j, "name": mujoco.mj_id2name(m, J, j), "type": int(m.jnt_type[j]),
        "qposadr": int(m.jnt_qposadr[j]), "dofadr": int(m.jnt_dofadr[j]),
        "body": mujoco.mj_id2name(m, B, int(m.jnt_bodyid[j])),
    })
hinges = [jj for jj in joints if jj["type"] != int(mujoco.mjtJoint.mjJNT_FREE)]
act_joint = [mujoco.mj_id2name(m, J, int(m.actuator_trnid[a, 0])) for a in range(m.nu)]
out["model"] = {
    "mjcf": mjcf, "xml_timestep": float(m.opt.timestep),
    "nq": m.nq, "nv": m.nv, "nu": m.nu, "njnt": m.njnt, "nbody": m.nbody, "ngeom": m.ngeom,
    "nsensor": m.nsensor, "nsite": m.nsite, "nmocap": m.nmocap,
    "joints": joints, "actuator_joint": act_joint,
    "actuator_gear0": [float(g) for g in m.actuator_gear[:, 0]],
    "actuator_ctrllimited": [int(x) for x in m.actuator_ctrllimited],
    "actuator_forcerange": m.actuator_forcerange.tolist(),
}
check("joint0 is free at qpos0/dof0", joints[0]["type"] == int(mujoco.mjtJoint.mjJNT_FREE)
      and joints[0]["qposadr"] == 0 and joints[0]["dofadr"] == 0 and joints[0]["body"] == "trunk")
check("hinge qpos order == RobotCfg.joint_names", [h["name"] for h in hinges] == list(rc.joint_names))
check("hinge qposadr contiguous 7..", [h["qposadr"] for h in hinges] == list(range(7, 7 + len(hinges))))
check("hinge dofadr contiguous 6..", [h["dofadr"] for h in hinges] == list(range(6, 6 + len(hinges))))
check("actuator order == RobotCfg.joint_names", act_joint == list(rc.joint_names))
check("nu == nq-7 == nv-6 == 23", m.nu == m.nq - 7 == m.nv - 6 == 23)
check("all actuator gear == 1", all(abs(g - 1.0) < 1e-12 for g in m.actuator_gear[:, 0]))

# sensors
sensors = []
for s in range(m.nsensor):
    sensors.append({
        "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, s),
        "type": int(m.sensor_type[s]), "adr": int(m.sensor_adr[s]), "dim": int(m.sensor_dim[s]),
        "noise_attr": float(m.sensor_noise[s]),
    })
out["model"]["sensors"] = sensors

# geoms: who collides, groups (LiDAR filter design input for W08)
robot_root = mujoco.mj_name2id(m, B, "trunk")
geoms = []
for g in range(m.ngeom):
    b = int(m.geom_bodyid[g])
    geoms.append({
        "id": g, "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g),
        "body": mujoco.mj_id2name(m, B, b), "robot": int(m.body_rootid[b]) == robot_root,
        "type": int(m.geom_type[g]), "group": int(m.geom_group[g]),
        "contype": int(m.geom_contype[g]), "conaffinity": int(m.geom_conaffinity[g]),
    })
out["model"]["geoms"] = geoms
summary: dict = {}
for gg in geoms:
    key = f"robot={gg['robot']} group={gg['group']} contype={gg['contype']} conaff={gg['conaffinity']}"
    summary[key] = summary.get(key, 0) + 1
out["model"]["geom_summary"] = summary

# --- kinematic frame probe: no mj_step ---------------------------------------
d = mujoco.MjData(m)
mujoco.mj_resetData(m, d)
yaw = np.pi / 2
d.qpos[:3] = cfg.mujoco.init_pos
d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
d.qpos[7:] = rc.default_joint_pos
d.qvel[:] = 0
d.qvel[0:3] = [1.0, 0.0, 0.0]
d.qvel[3:6] = [0.0, 0.0, 0.5]
mujoco.mj_forward(m, d)
# mjOBJ_XBODY = body frame (what qpos[0:7] describes); mjOBJ_BODY would be
# the inertial/COM frame, which differs by the 0.055 m COM offset and tilt.
XB = mujoco.mjtObj.mjOBJ_XBODY
v6_local = np.zeros(6)
mujoco.mj_objectVelocity(m, d, XB, robot_root, v6_local, 1)
v6_world = np.zeros(6)
mujoco.mj_objectVelocity(m, d, XB, robot_root, v6_world, 0)
v6_com_world = np.zeros(6)
mujoco.mj_objectVelocity(m, d, B, robot_root, v6_com_world, 0)
gyro = [s for s in sensors if s["name"] == "angular-velocity"][0]
quat_s = [s for s in sensors if s["name"] == "orientation"][0]
gyro_val = d.sensordata[gyro["adr"]:gyro["adr"] + 3].copy()
quat_val = d.sensordata[quat_s["adr"]:quat_s["adr"] + 4].copy()
out["frame_probe"] = {
    "setup": "objtype=mjOBJ_XBODY(trunk); trunk yaw=+90deg, qvel[0:3]=[1,0,0], qvel[3:6]=[0,0,0.5], mj_forward only",
    "objectVelocity_local_[ang,lin]": v6_local.round(6).tolist(),
    "objectVelocity_world_[ang,lin]": v6_world.round(6).tolist(),
    "objectVelocity_COM_world_[ang,lin]": v6_com_world.round(6).tolist(),
    "gyro_sensordata": gyro_val.round(6).tolist(),
    "framequat_sensordata": quat_val.round(6).tolist(),
    "qpos_quat": d.qpos[3:7].round(6).tolist(),
}
check("qvel[0:3] is WORLD-frame linear vel (local lin ~ [0,-1,0])", np.allclose(v6_local[3:], [0, -1, 0], atol=1e-6))
check("qvel[3:6] is BODY-frame angular vel", np.allclose(v6_local[:3], [0, 0, 0.5], atol=1e-6))
check("gyro sensor == qvel[3:6] (noise attr not applied)", np.allclose(gyro_val, d.qvel[3:6], atol=1e-9))
check("framequat == qpos[3:7]", np.allclose(quat_val, d.qpos[3:7], atol=1e-9))

# --- policy I/O (TorchScript load, zero obs, no controller) -------------------
ckpt = os.path.join(UPSTREAM, "tasks/locomotion", cfg.policy.checkpoint_path)
model = torch.jit.load(ckpt, map_location="cpu").eval()
n_pj = len(cfg.policy.policy_joint_names)
obs_dim = 3 + 3 + 3 + 3 * n_pj
with torch.inference_mode():
    act = model(torch.zeros(obs_dim * cfg.policy.actor_obs_history_length))
out["policy_io"] = {
    "checkpoint": ckpt, "frame_obs_dim": obs_dim,
    "frame_layout": "ang_vel_b(3) proj_gravity(3) cmd[vx,vy,wz](3) dq(21) dqd(21) last_action(21)",
    "input_dim": obs_dim * cfg.policy.actor_obs_history_length, "output_dim": int(act.numel()),
}
check("policy output == 21", act.numel() == n_pj == 21)

# --- MuJoCo API surface used by later steps ----------------------------------
api = {}
for name in ("mj_ray", "mj_multiRay", "mj_objectVelocity", "mj_resetData", "mj_forward", "mj_step", "mj_name2id"):
    fn = getattr(mujoco, name, None)
    api[name] = (fn.__doc__ or "").strip().splitlines()[0] if fn else None
api["MjSpec"] = hasattr(mujoco, "MjSpec")
api["MjSpec.from_file"] = hasattr(getattr(mujoco, "MjSpec", None), "from_file")
api["MjSpec.attach"] = hasattr(getattr(mujoco, "MjSpec", None), "attach")
api["MjSpec.compile"] = hasattr(getattr(mujoco, "MjSpec", None), "compile")
api["mujoco_version"] = mujoco.__version__
out["mujoco_api"] = api
check("mj_multiRay available", api["mj_multiRay"] is not None)
check("MjSpec from_file/attach/compile available", api["MjSpec.from_file"] and api["MjSpec.attach"] and api["MjSpec.compile"])

out["checks"] = checks
out["all_pass"] = all(checks.values())
out["finished_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
text = json.dumps(out, indent=1, ensure_ascii=False)
if len(sys.argv) > 1:
    with open(sys.argv[1], "w", encoding="utf-8") as f:
        f.write(text + "\n")
for k, v in checks.items():
    print(("PASS " if v else "FAIL ") + k)
print("ALL_PASS" if out["all_pass"] else "SOME_FAIL")
sys.exit(0 if out["all_pass"] else 1)

"""Single-owner deterministic controller. Load task config before importing."""
import math
import threading
import time
from queue import Empty

import mujoco
import numpy as np
import torch
from booster_deploy.controllers.mujoco_controller import MujocoController
from .model_contract import joint_map, ModelIncompatible
from .control import InputBuffer, RequestQueue


class SimulationFault(RuntimeError):
    pass


class T1SimRunner(MujocoController):
    def __init__(self, cfg, *, debug=False, warmup=2.0, timeout_sim=0.5, timeout_wall=0.5):
        self.owner = threading.get_ident()
        if (cfg.policy_dt, cfg.mujoco.decimation, cfg.mujoco.physics_dt) != (0.02, 10, 0.002):
            raise ModelIncompatible('frozen timing must be 0.02/10/0.002')
        # Upstream initializes qpos before it can be checked: validate first.
        model = mujoco.MjModel.from_xml_path(self._expand_assets_placeholder(cfg.robot.mjcf_path))
        self.qadr, self.vadr, self.aids = joint_map(model, cfg.robot.joint_names)
        del model
        super().__init__(cfg)
        self.inputs = InputBuffer()
        self.requests = RequestQueue()
        self.debug, self.warmup = debug, warmup
        self.timeout_sim, self.timeout_wall = timeout_sim, timeout_wall
        self.state, self.fault = 'STARTING', None
        self.episode = 0
        self.episode_start = 0.0
        self.u_pol = (0.0, 0.0, 0.0)
        self.incoming = self.u_pol
        self.cmd_status = 'BLOCKED_BY_STATE'
        self.action_norm = 0.0
        self.compute_ms = 0.0
        self.state_events = []
        self.had_failure = False
        self.update_state()
        self.start()
        self.transition('STARTING')

    def assert_owner(self):
        if threading.get_ident() != self.owner:
            raise RuntimeError('MuJoCo access outside the simulation owner thread')

    def update_vel_command(self):
        """No stdin. Only the owner writes VelocityCommand at step boundaries."""

    def write_command(self, command):
        self.assert_owner()
        self.u_pol = tuple(command)
        (self.vel_command.lin_vel_x, self.vel_command.lin_vel_y,
         self.vel_command.ang_vel_yaw) = self.u_pol

    def transition(self, state, fault=None):
        self.assert_owner()
        self.state, self.fault = state, fault
        self.inputs.boundary(state, float(self.mj_data.time), clear=True)
        self.write_command((0.0, 0.0, 0.0))
        self.incoming = self.u_pol
        self.cmd_status = 'BLOCKED_BY_STATE'
        self.state_events.append((state, fault, float(self.mj_data.time), self.episode))
        if state in ('FALLEN', 'FAULT'):
            self.had_failure = True
            self.stop()

    def update_state(self):
        self.assert_owner()
        d, r = self.mj_data, self.robot.data
        for name, array in (
            ('joint_pos', d.qpos[self.qadr]), ('joint_vel', d.qvel[self.vadr]),
            ('feedback_torque', d.qfrc_actuator[self.vadr]), ('root_pos_w', d.qpos[:3]),
            ('root_quat_w', d.qpos[3:7]), ('root_ang_vel_b', d.qvel[3:6]),
        ):
            setattr(r, name, torch.from_numpy(array.astype(np.float32)).to(r.device))
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, d.qpos[3:7])
        r.root_lin_vel_b = torch.from_numpy((rotation.reshape(3, 3).T @ d.qvel[:3]).astype(np.float32)).to(r.device)

    def finite_state(self):
        return all(np.isfinite(x).all() for x in (self.mj_data.qpos, self.mj_data.qvel, self.mj_data.qacc))

    def ctrl_step(self, dof_targets):
        self.assert_owner()
        targets = dof_targets.detach().cpu().numpy()
        kp, kd, limit = (x.numpy() for x in
                         (self.robot.joint_stiffness, self.robot.joint_damping, self.robot.effort_limit))
        for _ in range(self.decimation):
            q = self.mj_data.qpos[self.qadr].astype(np.float32)
            v = self.mj_data.qvel[self.vadr].astype(np.float32)
            torque = np.clip(kp * (targets - q) - kd * v, -limit, limit)
            if not np.isfinite(torque).all():
                raise SimulationFault('nonfinite_action')
            self.mj_data.ctrl[self.aids] = torque
            mujoco.mj_step(self.mj_model, self.mj_data)
            if not self.finite_state():
                raise SimulationFault('nonfinite_state')

    def reset_episode(self):
        self.assert_owner()
        next_time = float(self.mj_data.time) + self.cfg.policy_dt
        self.transition('RESETTING')
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        self.mj_data.qpos[:] = np.concatenate([
            np.asarray(self.cfg.mujoco.init_pos, dtype=np.float32),
            np.asarray(self.cfg.mujoco.init_quat, dtype=np.float32),
            self.robot.default_joint_pos.numpy()])
        self.mj_data.time = next_time
        mujoco.mj_forward(self.mj_model, self.mj_data)
        self.episode += 1
        self.episode_start = next_time
        self.action_norm = self.compute_ms = 0.0
        self.update_state()
        self.start()  # resets history, last_action, filter, counters
        self.transition('STARTING')

    def handle_requests(self, mode):
        self.assert_owner()
        while True:
            try:
                request = self.requests.queue.get_nowait()
            except Empty:
                break
            with request.lock:
                if request.cancelled or time.monotonic() > request.deadline:
                    request.result = (False, 'request expired; not applied')
                elif request.operation == 'reset':
                    if mode != 'sim_only':
                        request.result = (False, 'restart the entire launch outside sim_only')
                    else:
                        self.reset_episode()
                        request.result = (True, f'episode={self.episode}; clock remains monotonic')
                elif request.operation == 'pause':
                    if self.state not in ('STARTING', 'READY', 'PAUSED'):
                        request.result = (False, f'cannot pause/resume {self.state}')
                    else:
                        if request.value and self.state != 'PAUSED':
                            self.resume_state = self.state
                            self.transition('PAUSED')
                        elif not request.value and self.state == 'PAUSED':
                            self.transition(self.resume_state)
                        request.result = (True, self.state)
                else:
                    request.result = (False, 'unknown request')
                request.done.set()

    def step(self):
        self.assert_owner()
        if self.state not in ('STARTING', 'READY'):
            return False
        began = time.monotonic()
        if not self.finite_state():
            self.transition('FAULT', 'nonfinite_state')
            return False
        self.incoming, command, self.cmd_status = self.inputs.consume(
            float(self.mj_data.time), began, self.timeout_sim, self.timeout_wall, self.debug)
        self.write_command(command)
        self.update_state()
        try:
            target = self.policy_step()
        except Exception as exc:
            self.transition('FAULT', 'policy_exception')
            self.failure_detail = repr(exc)
            return False
        if not self.is_running:
            self.transition('FALLEN')
            return False
        if target.shape != (23,) or not torch.isfinite(target).all():
            self.transition('FAULT', 'nonfinite_action')
            return False
        self.action_norm = float(torch.linalg.vector_norm(self.policy.last_action))
        try:
            self.ctrl_step(target)
        except SimulationFault as exc:
            self.transition('FAULT', str(exc))
            return False
        self.update_state()
        self.compute_ms = (time.monotonic() - began) * 1000
        self.inputs.boundary(self.state, float(self.mj_data.time))
        return True

    def finish_boundary(self):
        # Called after recording the just-completed step, before new commands.
        if self.state == 'STARTING' and self.mj_data.time - self.episode_start + 1e-9 >= self.warmup:
            self.transition('READY')

    def measurements(self):
        d = self.mj_data
        w, x, y, z = d.qpos[3:7]
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, d.qpos[3:7])
        vx, vy = d.qvel[:2]
        return dict(x_w=float(d.qpos[0]), y_w=float(d.qpos[1]), z_w=float(d.qpos[2]),
                    yaw_w=yaw, roll=roll, pitch=pitch, proj_grav_z=-(1-2*(x*x+y*y)),
                    v_meas_x=math.cos(yaw)*vx+math.sin(yaw)*vy,
                    v_meas_y=-math.sin(yaw)*vx+math.cos(yaw)*vy,
                    v_meas_wz=float((rotation.reshape(3,3) @ d.qvel[3:6])[2]))

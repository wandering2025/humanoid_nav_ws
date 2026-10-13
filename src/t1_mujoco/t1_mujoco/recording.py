"""Ignored per-run artifacts with reproducible provenance and physical trace."""
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import uuid


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(root, *args):
    try:
        return subprocess.check_output(['git', '-C', str(root), *args], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unavailable'


class RunLog:
    def __init__(self, params, runtime):
        import mujoco
        import torch
        self.started = time.monotonic()
        self.run_id = params['run_id'] or (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '_dev_w04_s' + str(params['seed']) + '_' + uuid.uuid4().hex[:8])
        self.path = Path(params['run_dir']) / self.run_id
        self.path.mkdir(parents=True, exist_ok=False)
        workspace = Path(os.environ.get('T1_WS', Path.cwd()))
        dirty = git(workspace, 'status', '--short')
        self.manifest = dict(
            schema=1, run_id=self.run_id, created_utc=datetime.now(timezone.utc).isoformat(),
            code_sha=git(workspace, 'rev-parse', 'HEAD'), dirty=bool(dirty), dirty_files=dirty.splitlines(),
            upstream_sha={name: git(path, 'rev-parse', 'HEAD') for name, path in (
                ('booster_deploy', params['upstream_path']), ('booster_assets', runtime['assets_path']))},
            checkpoint_sha256=sha256(runtime['checkpoint_path']), mjcf_sha256=sha256(runtime['mjcf_path']),
            scene_file=runtime['mjcf_path'], scene_sha256=sha256(runtime['mjcf_path']),
            mode=params['mode'], suite='dev', case='w04', seed=params['seed'],
            sensor_mode=None, odom_mode=None, map_yaml=None, map_sha256=None, T_world_map=None,
            goal=None, tolerances=None, time_limit_s=params['max_sim_time'],
            parameter_files={key: None for key in ('nav2','slam','amcl','velocity_smoother','collision_monitor','command_map')},
            resolved_parameters=params, parameters_sha256=hashlib.sha256(json.dumps(params,sort_keys=True).encode()).hexdigest(),
            headless=params['headless'], target_rtf=params['target_rtf'],
            policy_dt=runtime['policy_dt'], physics_dt=runtime['physics_dt'], decimation=runtime['decimation'],
            host=platform.node(), cpu=platform.processor(), python=sys.version, executable=sys.executable,
            ros=os.environ.get('ROS_DISTRO'), mujoco=mujoco.__version__, torch=torch.__version__,
            command_line=sys.argv, status='STARTING', exit_code=None,
        )
        self.trace_file = (self.path/'sim_trace.csv').open('w')
        fields = ('sim_time wall_mono episode state cmd_status cmd_source '
                  'v_cmd_in_x v_cmd_in_y v_cmd_in_wz v_guard_x v_guard_y v_guard_wz '
                  'u_pol_x u_pol_y u_pol_wz v_meas_x v_meas_y v_meas_wz x_w y_w z_w yaw_w roll pitch '
                  'proj_grav_z action_norm step_compute_ms scan_due scan_compute_ms rtf_10s nonfinite').split()
        self.trace = csv.DictWriter(self.trace_file, fieldnames=fields)
        self.trace.writeheader()
        self.events = (self.path/'events.jsonl').open('w')
        self.save()

    def save(self):
        (self.path/'manifest.json').write_text(json.dumps(self.manifest, indent=2, allow_nan=False)+'\n')

    def event(self, status):
        self.events.write(json.dumps(status, allow_nan=False)+'\n')
        self.events.flush()

    def row(self, runner, rtf):
        row = dict(sim_time=float(runner.mj_data.time), wall_mono=time.monotonic(), episode=runner.episode,
                   state=runner.state, cmd_status=runner.cmd_status,
                   cmd_source='policy_cmd' if runner.debug else 'cmd_vel',
                   action_norm=runner.action_norm, step_compute_ms=runner.compute_ms,
                   scan_due=False, scan_compute_ms=0, rtf_10s=rtf, nonfinite=False)
        row.update(runner.measurements())
        for i, axis in enumerate(('x','y','wz')):
            # Debug inputs are policy units, never label them as physical desired velocity.
            row['v_cmd_in_'+axis] = 0.0 if runner.debug else runner.incoming[i]
            row['v_guard_'+axis] = 0.0  # physical mapping is unavailable until W05
            row['u_pol_'+axis] = runner.u_pol[i]
        self.trace.writerow(row)

    def close(self, code, sim_time, status):
        self.manifest.update(exit_code=code, sim_time_end=sim_time,
                             wall_duration_s=time.monotonic()-self.started,
                             ended_utc=datetime.now(timezone.utc).isoformat(), status=status)
        self.save()
        self.trace_file.close()
        self.events.close()

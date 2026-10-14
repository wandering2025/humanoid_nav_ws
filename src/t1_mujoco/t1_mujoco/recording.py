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


def w09_provenance(params, scene):
    """Bind maps and configuration to logs; metadata never enters control/SLAM."""
    import yaml
    files = {}
    for name, key in (('slam', 'slam_params_file'), ('amcl', 'localization_params_file'),
                      ('nav2', 'navigation_params_file')):
        value = params.get(key, '')
        if value:
            files[name] = dict(path=value, sha256=sha256(value))
    value = params.get('map_file', '')
    result = dict(parameter_files=files)
    if value:
        path = Path(value)
        map_config = yaml.safe_load(path.read_text())
        meta = yaml.safe_load(path.with_suffix('.meta.yaml').read_text())
        image = path.parent / map_config['image']
        if sha256(path) != meta['map_yaml_sha256'] or sha256(image) != meta['map_image_sha256']:
            raise ValueError('saved map checksum does not match map metadata')
        if not scene or scene['layout']['sha256'] != meta['scene_yaml_sha256']:
            raise ValueError('localization/navigation scene does not match saved map metadata')
        result.update(map_yaml=str(path), map_sha256=sha256(path), map_image_sha256=sha256(image),
                      T_world_map=meta['T_world_map'], map_generated_code_sha=meta['generated_code_sha'])
    return result


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
            sensor_mode=None, odom_mode='ideal_truth', map_yaml=None, map_sha256=None, T_world_map=None,
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
                  'proj_grav_z action_norm step_compute_ms policy_compute_ms physics_compute_ms scan_due scan_compute_ms rtf_10s nonfinite').split()
        self.trace = csv.DictWriter(self.trace_file, fieldnames=fields)
        self.trace.writeheader()
        self.events = (self.path/'events.jsonl').open('w')
        # Offline evaluation snapshots only; never consumed by navigation.
        self.geometry = (self.path/'geometry.jsonl').open('w')
        self.geometry_step = 0
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
                   policy_compute_ms=runner.policy_compute_ms, physics_compute_ms=runner.physics_compute_ms,
                   scan_due=runner.scan_due, scan_compute_ms=runner.scan_compute_ms, rtf_10s=rtf, nonfinite=False)
        row.update(runner.measurements())
        for i, axis in enumerate(('x','y','wz')):
            # Debug inputs are policy units, never label them as physical desired velocity.
            row['v_cmd_in_'+axis] = 0.0 if runner.debug else runner.incoming[i]
            # Expected physical velocity of u_pol under the W05 map (M^-1).
            row['v_guard_'+axis] = 0.0 if runner.debug else runner.v_guard[i]
            row['u_pol_'+axis] = runner.u_pol[i]
        self.trace.writerow(row)
        if self.geometry_step % 5 == 0:  # 10 Hz, after the physical step
            d = runner.mj_data
            self.geometry.write(json.dumps(dict(sim_time=float(d.time), qpos=d.qpos.tolist(),
                mocap_pos=d.mocap_pos.tolist(), mocap_quat=d.mocap_quat.tolist())) + '\n')
        self.geometry_step += 1

    def close(self, code, sim_time, status):
        self.manifest.update(exit_code=code, sim_time_end=sim_time,
                             wall_duration_s=time.monotonic()-self.started,
                             ended_utc=datetime.now(timezone.utc).isoformat(), status=status)
        self.save()
        self.trace_file.close()
        self.events.close()
        self.geometry.close()

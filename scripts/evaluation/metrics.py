"""Read-only offline metrics on saved MuJoCo states; no ROS/control imports."""
import csv
import json
import math
from pathlib import Path

import numpy as np


def distribution(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(n=len(a), median=float(np.median(a)), p5=float(np.percentile(a, 5)),
                p95=float(np.percentile(a, 95)), max=float(a.max())) if len(a) else dict(n=0)


def trace_columns(path):
    with Path(path).open() as f:
        rows = list(csv.DictReader(f))
    return rows, {k: np.array([(1. if r[k] == 'True' else 0. if r[k] == 'False' else float(r[k])) for r in rows]) for k in rows[0] if k not in
                  ('state', 'cmd_status', 'cmd_source')}


def collision_geom_sets(model):
    root = model.body('trunk').id
    collision = (model.geom_contype | model.geom_conaffinity) != 0
    robot = model.body_rootid[model.geom_bodyid] == root
    ground = model.geom('ground').id
    return np.flatnonzero(robot & collision), np.flatnonzero(~robot & collision & (np.arange(model.ngeom) != ground))


def body_clearance(manifest, geometry):
    """10 Hz signed 3D distance of collision geoms, all environment except ground.

    Reads a separate MjData after the live process has exited. Sphere bounds only
    prune pairs whose lower bound already exceeds the best exact distance.
    """
    import mujoco
    model = mujoco.MjModel.from_xml_path(manifest['scene_file'])
    data = mujoco.MjData(model)
    robot, env = collision_geom_sets(model)
    if not len(robot) or not len(env):
        raise ValueError('missing robot/environment collision geometry')
    best, pair, t_best, n = float('inf'), None, None, 0
    with Path(geometry).open() as f:
        for line in f:
            row = json.loads(line)
            data.qpos[:] = row['qpos']
            if model.nmocap:
                data.mocap_pos[:] = row['mocap_pos']
                data.mocap_quat[:] = row['mocap_quat']
            mujoco.mj_forward(model, data)
            for a in robot:
                for b in env:
                    bound = np.linalg.norm(data.geom_xpos[a]-data.geom_xpos[b])-model.geom_rbound[a]-model.geom_rbound[b]
                    if bound > max(best, 0):
                        continue
                    distance = mujoco.mj_geomDistance(model, data, int(a), int(b), max(best, .001) if math.isfinite(best) else 100., None)
                    if distance < best:
                        best, pair, t_best = float(distance), [int(a), int(b)], row['sim_time']
            n += 1
    return dict(min_m=best, sim_time=t_best, geom_pair=[model.geom(i).name for i in pair], samples=n,
                robot_geoms=len(robot), environment_geoms=len(env), sample_hz=10,
                definition='signed 3D MuJoCo collision geometry distance; excludes ground/self/visual; sampled minimum, not continuous bound')


def stopping_metrics(rows, trace, events):
    """CM stop edges, subsequent zero policy, W05 .5 s stationary window.

    Displacement is maximum from first zero policy until first subsequent nonzero
    policy. Includes gait rebound; an unrecovered stop is measured to trace end.
    """
    t, x, y = (trace[k] for k in ('sim_time', 'x_w', 'y_w'))
    yaw = np.unwrap(trace['yaw_w'])
    speed, rate = np.full(len(t), np.inf), np.full(len(t), np.inf)
    speed[25:] = np.hypot(x[25:]-x[:-25], y[25:]-y[:-25])/.5
    rate[25:] = abs(yaw[25:]-yaw[:-25])/.5
    zero = (trace['u_pol_x'] == 0) & (trace['u_pol_y'] == 0) & (trace['u_pol_wz'] == 0)
    output = []
    for e in events:
        if e['kind'] != 'collision_monitor' or e['action_type'] != 1:
            continue
        i = int(np.searchsorted(t, e['sim']))
        if i >= len(t) or e['sim'] < 3 or not np.any(~zero[max(0,i-50):i]):
            continue  # retain startup events in original run, but not braking measures
        zeros = np.flatnonzero(zero[i:])
        if not len(zeros):
            output.append(dict(trigger_sim=e['sim'], zero_policy_sim=None)); continue
        j = i + int(zeros[0])
        resumes = np.flatnonzero(~zero[j:])
        k = j + int(resumes[0]) if len(resumes) else len(t)
        stationary = (speed < .05) & (rate < .05)
        stopped = next((q for q in range(j, k-100) if stationary[q:q+101].all()), None)
        output.append(dict(trigger_sim=e['sim'], zero_policy_sim=float(t[j]), zero_latency_s=float(t[j]-e['sim']),
             physical_stopped_sim=float(t[stopped]) if stopped is not None else None,
             resumed_sim=float(t[k]) if k < len(t) else None, trigger_speed_m_s=float(trace['v_meas_x'][i]),
             max_displacement_after_zero_m=float(np.hypot(x[j:k]-x[j], y[j:k]-y[j]).max())))
    return output


def classify(result, validity):
    """Deterministic precedence, with all underlying causes retained in JSON."""
    m = result.get('metrics', {})
    c = m.get('env_contacts', {})
    checks = result.get('validity_checks', {})
    protocol = ('source_clean', 'fresh_clock', 'monotonic_clock', 'frozen_timing')
    if checks and any(checks.get(k) is False for k in protocol):
        return 'INVALID'
    manifest = result.get('manifest') or result.get('run', {}).get('manifest', {})
    if manifest.get('status') == 'FALLEN':
        return 'FAIL_FALL'
    # Missing AMCL TF/initialpose is an observed bringup failure, not a bad
    # measurement protocol. Require intact source/time/manifest/cleanup evidence.
    startup = (not result.get('nav_active_sim') and result.get('error') and
               not result.get('metric_error') and
               all(checks.get(k) is True for k in (*protocol, 'complete_manifest', 'processes_reaped')))
    if startup:
        return 'FAIL_LOCALIZATION' if result.get('run', {}).get('initializations') else 'FAIL_STARTUP'
    if not validity:
        return 'INVALID'
    if m.get('fell') or c.get('ground_nonfoot', 0):
        return 'FAIL_FALL'
    if c.get('static_env', 0) or c.get('obstacle', 0):
        return 'FAIL_COLLISION'
    if m.get('settle_complete_task_s', 0) > result.get('time_limit_s', 120.):
        return 'FAIL_TIMEOUT'
    if result.get('outcome') == 'SUCCESS':
        return 'SUCCESS'
    if not result.get('nav_active_sim'):
        return 'FAIL_LOCALIZATION' if result.get('run', {}).get('initializations') else 'FAIL_STARTUP'
    if m.get('nav2_status') == 'ABORTED':
        return 'FAIL_PLANNING'
    if result.get('nav2', {}).get('timed_out'):
        return 'FAIL_TIMEOUT'
    if m.get('safety', {}).get('gate_stopped', 0):
        return 'FAIL_SAFETY_STOP_UNRECOVERED'
    return 'FAIL_LOCALIZATION' if m.get('nav2_status') == 'SUCCEEDED' and not m.get('arrived_truth') else 'FAIL_PLANNING'


def resolve_tf_authorities(records):
    """Resolve startup unknown names by stable RMW GID without hiding duplicates."""
    names, child_gids = {}, {}
    unknown = {'unresolved', '_NODE_NAME_UNKNOWN_'}
    for r in records:
        child_gids.setdefault(r['child'], set()).add(r['gid'])
        if r['node'] not in unknown:
            names.setdefault(r['gid'], set()).add(r['node'])
    owners = {child: sorted({name for gid in gids for name in names.get(gid, {'unresolved:'+gid})})
              for child, gids in child_gids.items()}
    return owners, {child: sorted(gids) for child, gids in child_gids.items()}

"""W05 scripted locomotion trials. Physical motion comes only from the policy.

A trial builds a fresh T1SimRunner, optionally rotates the *initial* world
heading (a symmetry of the flat plane; joints, height and warmup unchanged),
then feeds policy-unit commands through the same InputBuffer path that ROS uses.
All rates are computed from data.time, never wall time.
"""
from dataclasses import dataclass, field
import math

import numpy as np

POLICY_DT = 0.02
TRACE_FIELDS = ('trial', 'rep', 'k', 'sim_time', 'segment', 'state', 'cmd_status',
                'u_pol_x', 'u_pol_y', 'u_pol_wz', 'x_w', 'y_w', 'z_w', 'yaw_w', 'roll', 'pitch',
                'proj_grav_z', 'v_meas_x', 'v_meas_y', 'v_meas_wz', 'action_norm',
                'left_foot', 'right_foot', 'nonfoot_ground')


@dataclass
class Segment:
    duration: float
    command: tuple  # policy units (ux, uy, uwz)

    @property
    def steps(self):
        return int(round(self.duration / POLICY_DT))


@dataclass
class TrialSpec:
    name: str
    segments: list
    heading_deg: float = 0.0
    warmup: float = 2.0
    rep: int = 0
    tags: dict = field(default_factory=dict)


def yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class ContactProbe:
    """Read-only contact classification on the owner thread."""

    def __init__(self, model):
        import mujoco
        self.mujoco = mujoco
        self.ground = model.geom('ground').id if _has_geom(model, 'ground') else 0
        foot_bodies = {model.body(n).id for n in ('left_ankle_roll_link', 'right_ankle_roll_link')}
        self.left = model.body('left_ankle_roll_link').id
        self.right = model.body('right_ankle_roll_link').id
        self.model, self.foot_bodies = model, foot_bodies

    def sample(self, data):
        left = right = nonfoot = 0
        for i in range(data.ncon):
            c = data.contact[i]
            if c.dist > 0:
                continue
            pair = (c.geom1, c.geom2)
            if self.ground not in pair:
                continue
            other = pair[1] if pair[0] == self.ground else pair[0]
            body = self.model.geom_bodyid[other]
            if body == self.left:
                left = 1
            elif body == self.right:
                right = 1
            else:
                nonfoot += 1
        return left, right, nonfoot


def _has_geom(model, name):
    try:
        model.geom(name)
        return True
    except KeyError:
        return False


def run_trial(cfg, spec, *, record=None):
    """Return a structured array-like dict of per-step samples."""
    import mujoco
    from .runner import T1SimRunner
    r = T1SimRunner(cfg, debug=True, warmup=spec.warmup)
    if spec.heading_deg:
        # Initial condition only, before the first physics step.
        r.mj_data.qpos[3:7] = yaw_quat(math.radians(spec.heading_deg))
        mujoco.mj_forward(r.mj_model, r.mj_data)
        r.update_state()
    probe = ContactProbe(r.mj_model)
    rows = []
    # STARTING: the runner forces zero command until warmup elapses.
    while r.state == 'STARTING':
        if not r.step():
            break
        r.finish_boundary()
    k = 0
    for index, segment in enumerate(spec.segments):
        ux, uy, uwz = segment.command
        for _ in range(segment.steps):
            if r.state != 'READY':
                break
            t_cmd = float(r.mj_data.time)
            # Fresh message every step; wall stamp is "now" so timeouts never fire.
            r.inputs.receive((ux, uy, 0.0, 0.0, 0.0, uwz))
            ok = r.step()
            m = r.measurements()
            left, right, nonfoot = probe.sample(r.mj_data)
            row = dict(trial=spec.name, rep=spec.rep, k=k, sim_time=float(r.mj_data.time),
                       t_cmd=t_cmd, segment=index, state=r.state, cmd_status=r.cmd_status,
                       u_pol_x=r.u_pol[0], u_pol_y=r.u_pol[1], u_pol_wz=r.u_pol[2],
                       action_norm=r.action_norm, left_foot=left, right_foot=right,
                       nonfoot_ground=nonfoot, **m)
            rows.append(row)
            if record is not None:
                record(row)
            k += 1
            if not ok:
                break
            r.finish_boundary()
    result = dict(spec=spec, rows=rows, final_state=r.state, fault=r.fault,
                  sim_end=float(r.mj_data.time))
    r.stop()
    return result


def _arr(rows, key):
    return np.array([row[key] for row in rows], dtype=float)


def stillness(rows, window_s=0.5):
    """Windowed base_footprint speed (m/s) and yaw rate (rad/s) per row.

    Instantaneous trunk velocity oscillates with stance sway, so stillness
    uses displacement over a trailing window: mean planar speed.
    """
    n = max(1, int(round(window_s / POLICY_DT)))
    x, y, yaw = _arr(rows, 'x_w'), _arr(rows, 'y_w'), np.unwrap(_arr(rows, 'yaw_w'))
    speed = np.full(len(rows), np.nan)
    rate = np.full(len(rows), np.nan)
    if len(rows) > n:
        speed[n:] = np.hypot(x[n:] - x[:-n], y[n:] - y[:-n]) / (n * POLICY_DT)
        rate[n:] = np.abs(yaw[n:] - yaw[:-n]) / (n * POLICY_DT)
    return speed, rate


def summarize(result, *, still_speed=0.05, still_rate=0.05, settle_hold=2.0,
              steady_s=5.0):
    """Per-segment metrics; pure function of the recorded rows."""
    rows, spec = result['rows'], result['spec']
    out = []
    if not rows:
        return out
    speed, rate = stillness(rows)
    yaw_all = np.unwrap(_arr(rows, 'yaw_w'))
    seg = _arr(rows, 'segment').astype(int)
    hold = int(round(settle_hold / POLICY_DT))
    prev_moving = False
    for index, segment in enumerate(spec.segments):
        idx = np.flatnonzero(seg == index)
        if not len(idx):
            continue
        sub = [rows[i] for i in idx]
        t = _arr(sub, 'sim_time')
        t0 = sub[0]['t_cmd']
        x, y, z = _arr(sub, 'x_w'), _arr(sub, 'y_w'), _arr(sub, 'z_w')
        yaw = yaw_all[idx]
        vx, vy, wz = _arr(sub, 'v_meas_x'), _arr(sub, 'v_meas_y'), _arr(sub, 'v_meas_wz')
        tilt = np.degrees(np.arccos(np.clip(-_arr(sub, 'proj_grav_z'), -1, 1)))
        # Steady window: final steady_s seconds, at most half the segment.
        ns = min(len(sub), max(1, int(round(min(steady_s, segment.duration / 2) / POLICY_DT))))
        w = slice(len(sub) - ns, len(sub))
        # Reference pose = state at command onset (previous row, or first row).
        i0 = idx[0] - 1 if idx[0] > 0 else idx[0]
        x0, y0, yaw0 = rows[i0]['x_w'], rows[i0]['y_w'], yaw_all[i0]
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        fwd, lat = c * dx + s * dy, -s * dx + c * dy
        # Mean body-frame speed over the steady window from displacement along
        # the instantaneous heading (equivalent to averaging v_meas).
        steady = dict(vx=float(np.mean(vx[w])), vy=float(np.mean(vy[w])),
                      wz=float((yaw[-1] - yaw[len(sub) - ns]) / (ns * POLICY_DT)) if ns > 1 else float('nan'))
        ux, uy, uwz = segment.command
        moving = any(abs(v) > 0 for v in segment.command)
        item = dict(trial=spec.name, rep=spec.rep, heading_deg=spec.heading_deg,
                    segment=index, ux=ux, uy=uy, uwz=uwz, duration=segment.duration,
                    t0=t0, t_end=float(t[-1]), completed=len(sub) == segment.steps,
                    steady_vx=steady['vx'], steady_vy=steady['vy'], steady_wz=steady['wz'],
                    disp_fwd=float(fwd[-1]), disp_lat=float(lat[-1]),
                    yaw_change_deg=float(math.degrees(yaw[-1] - yaw0)),
                    max_abs_yaw_dev_deg=float(np.max(np.abs(np.degrees(yaw - yaw0)))),
                    max_tilt_deg=float(np.max(tilt)), min_z=float(np.min(z)), max_z=float(np.max(z)),
                    mean_action_norm=float(np.mean(_arr(sub, 'action_norm'))),
                    single_support_frac=float(np.mean((_arr(sub, 'left_foot') + _arr(sub, 'right_foot')) == 1)),
                    flight_frac=float(np.mean((_arr(sub, 'left_foot') + _arr(sub, 'right_foot')) == 0)),
                    nonfoot_ground=int(np.sum(_arr(sub, 'nonfoot_ground') > 0)),
                    final_state=sub[-1]['state'])
        if moving:
            item.update(_onset(sub, idx, speed, rate, steady, segment, t0, fwd, lat, yaw - yaw0))
        else:
            item.update(onset_move_s=float('nan'), onset_t90_s=float('nan'))
        if not moving and prev_moving:
            # Stop metrics: distance/time from zero-command onset until the
            # robot is still and stays still for settle_hold seconds.
            sp, rt = speed[idx], rate[idx]
            still = (sp < still_speed) & (rt < still_rate)
            settle = None
            for i in range(len(sub) - hold + 1):
                if still[i:i + hold].all():
                    settle = i
                    break
            dist = np.hypot(dx, dy)
            item['stop_time_s'] = float(t[settle] - t0) if settle is not None else float('nan')
            # Window speed lags; report the farthest excursion and the rest offset.
            item['stop_dist_max_m'] = float(np.max(dist))
            item['stop_dist_rest_m'] = float(dist[settle]) if settle is not None else float('nan')
            item['stop_fwd_m'] = float(fwd[settle]) if settle is not None else float('nan')
            item['stop_yaw_deg'] = float(math.degrees(yaw[settle] - yaw0)) if settle is not None else float('nan')
            item['settled_hold'] = settle is not None
        out.append(item)
        prev_moving = moving
    return out


def _onset(sub, idx, speed, rate, steady, segment, t0, fwd, lat, dyaw,
           move_m=0.05, move_rad=0.05, window_s=0.5):
    """Start latency on the dominant commanded axis (sim seconds after t0).

    onset_move_s: first time the pose has moved move_m (or yawed move_rad).
    onset_t90_s: first time the trailing-window rate reaches 90% of the steady
    value, minus half the window (window-centre correction).
    """
    ux, uy, uwz = segment.command
    t = _arr(sub, 'sim_time')
    if abs(ux) > 0 and abs(ux) >= abs(uy):
        series, rates, target, limit = np.abs(fwd), speed[idx], abs(steady['vx']), move_m
    elif abs(uy) > 0:
        series, rates, target, limit = np.abs(lat), speed[idx], abs(steady['vy']), move_m
    else:
        series, rates, target, limit = np.abs(dyaw), rate[idx], abs(steady['wz']), move_rad
    hit = np.flatnonzero(series >= limit)
    move = float(t[hit[0]] - t0) if len(hit) else float('nan')
    t90 = float('nan')
    if math.isfinite(target) and target > 1e-3:
        hit = np.flatnonzero(np.nan_to_num(rates) >= 0.9 * target)
        if len(hit):
            t90 = float(max(0.0, t[hit[0]] - t0 - window_s / 2))
    return dict(onset_move_s=move, onset_t90_s=t90)

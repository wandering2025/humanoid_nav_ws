import math

import pytest

from t1_mujoco.calibration import Segment, TrialSpec, summarize

DT = 0.02


def synthetic(heading_deg, speed=0.4, stop_after=0.3):
    """1 s still, 10 s at `speed` along the heading, then a 0.3 s linear brake."""
    spec = TrialSpec('syn', [Segment(1.0, (0, 0, 0)), Segment(10.0, (0.5, 0, 0)), Segment(6.0, (0, 0, 0))],
                     heading_deg=heading_deg)
    yaw = math.radians(heading_deg)
    rows, x, t = [], 0.0, 2.0
    for seg, steps, v_fn in ((0, 50, lambda i: 0.0), (1, 500, lambda i: speed),
                             (2, 300, lambda i: max(0.0, speed * (1 - (i + 1) * DT / stop_after)))):
        for i in range(steps):
            v = v_fn(i)
            x += v * DT
            rows.append(dict(sim_time=t + DT, t_cmd=t, segment=seg, state='READY',
                             x_w=math.cos(yaw) * x, y_w=math.sin(yaw) * x, z_w=0.68, yaw_w=yaw,
                             roll=0.0, pitch=0.0, proj_grav_z=-1.0, v_meas_x=v, v_meas_y=0.0,
                             v_meas_wz=0.0, action_norm=1.0, left_foot=1, right_foot=0,
                             nonfoot_ground=0))
            t += DT
    return dict(spec=spec, rows=rows)


@pytest.mark.parametrize('heading', [0.0, 90.0, 180.0, -135.0])
def test_metrics_are_body_frame_and_heading_invariant(heading):
    moving, stop = summarize(synthetic(heading))[1:]
    assert moving['steady_vx'] == pytest.approx(0.4)
    assert moving['disp_fwd'] == pytest.approx(4.0)
    assert abs(moving['disp_lat']) < 1e-9
    assert moving['onset_move_s'] == pytest.approx(0.14, abs=0.021)  # 0.05 m at 0.4 m/s
    assert moving['single_support_frac'] == 1.0
    # Brake distance = v*T/2 = 0.06 m; still within 0.3 s plus window lag.
    assert stop['stop_dist_max_m'] == pytest.approx(0.06, abs=0.005)
    assert stop['settled_hold'] and stop['stop_time_s'] < 0.8


def test_no_stop_metrics_after_zero_segment():
    items = summarize(synthetic(0.0))
    assert 'stop_time_s' not in items[0] and 'stop_time_s' not in items[1]

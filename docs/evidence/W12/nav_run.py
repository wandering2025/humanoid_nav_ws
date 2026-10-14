import os
"""W11 navigation episodes: full navigation.launch.py, one goal via NavigateToPose.

Each run is a fresh launch (new ROS graph, /clock from 0), start pose set before
the runner exists, AMCL initialized once with the case's nominal start pose. The
goal is sent with the user-facing `ros2 run t1_bringup navigate_to_pose` tool (or
this probe's own action client for cancel/replace). Metrics use t1_sim's 50 Hz
sim_trace (truth) only for evaluation, never for control.

  nav_run.py dev  --start "x y yaw" --goal "x y yaw" [--label L] [--rviz]
  nav_run.py case S01            (one predefined case from w12_static_cases.yaml)
  nav_run.py suite               (all predefined cases, in file order)
  nav_run.py cancel              (cancel + explicit re-arm + preemptive replacement)
"""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import time

import numpy as np
import yaml
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger

from harness import LAYERS, OUT, RAW, WS, Run, Session, angle, digest, git

CASES = WS/'src/t1_navigation/config/w11_static_cases.yaml'
SCENE = WS/f'src/t1_mujoco/worlds/{os.environ.get("W12_SCENE", "dynamic_crossing_world")}.yaml'
FOOTPRINT = WS/'src/t1_description/config/footprint.yaml'
META = WS/f'src/t1_navigation/maps/{os.environ.get("W12_SCENE", "dynamic_crossing_world")}.meta.yaml'
LOG_PATTERNS = dict(tf_extrapolation=r'xtrapolation', tf_old_data=r'TF_OLD_DATA', tf_timeout=r'Timed out waiting for transform',
                    no_valid_control=r'[Nn]o valid|Optimizer fail|failed to find', progress_failure=r'Failed to make progress',
                    planning_failure=r'Failed to create plan|GridBased plugin failed', goal_aborted=r'[Gg]oal .*aborted|Aborting',
                    collision_ahead=r'[Cc]ollision ahead|Robot is in collision', scan_dropped=r'[Dd]ropp')


def scene_rectangles():
    data = yaml.safe_load(SCENE.read_text())
    xmin, xmax, ymin, ymax = data['bounds']
    t = .2
    rects = [(xmin-t, xmin, ymin-t, ymax+t), (xmax, xmax+t, ymin-t, ymax+t), (xmin, xmax, ymin-t, ymin), (xmin, xmax, ymax, ymax+t)]
    for b in data['boxes']:
        (x, y, _), (sx, sy, _) = b['center'], b['size']
        rects.append((x-sx/2, x+sx/2, y-sy/2, y+sy/2))
    return rects


def rect_distance(points, r):
    x0, x1, y0, y1 = r
    dx = np.maximum.reduce([x0-points[:, 0], points[:, 0]-x1, np.zeros(len(points))])
    dy = np.maximum.reduce([y0-points[:, 1], points[:, 1]-y1, np.zeros(len(points))])
    return np.hypot(dx, dy)


def footprint_points():
    poly = np.array(yaml.safe_load(FOOTPRINT.read_text())['polygon'], float)
    pts = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        n = max(2, int(np.linalg.norm(b-a)/.02))
        pts.extend(a + (b-a)*t for t in np.linspace(0, 1, n, endpoint=False))
    return np.array(pts), poly


def inside_polygon(p, poly):
    x, y, inside = p[0], p[1], False
    for (x1, y1), (x2, y2) in zip(poly, np.roll(poly, -1, axis=0)):
        if (y1 > y) != (y2 > y) and x < (x2-x1)*(y-y1)/(y2-y1) + x1:
            inside = not inside
    return inside


def clearances(trace, rects):
    xy = np.c_[trace['x_w'], trace['y_w']]
    center = np.min([rect_distance(xy, r) for r in rects], axis=0)
    local, poly = footprint_points()
    foot = []
    for i in range(0, len(trace['x_w']), 5):  # 10 Hz
        c, s = math.cos(trace['yaw_w'][i]), math.sin(trace['yaw_w'][i])
        R = np.array([[c, -s], [s, c]])
        pts = local @ R.T + xy[i]
        d = min(float(rect_distance(pts, r).min()) for r in rects)
        world_poly = poly @ R.T + xy[i]
        for x0, x1, y0, y1 in rects:
            if any(inside_polygon(q, world_poly) for q in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))):
                d = 0.
        foot.append(d)
    return float(center.min()), float(min(foot))


def load_trace(run_id):
    path = WS/'run_data'/run_id/'sim_trace.csv'
    with path.open() as f:
        rows = list(csv.DictReader(f))
    keys = ('sim_time', 'x_w', 'y_w', 'z_w', 'yaw_w', 'v_meas_x', 'v_meas_wz', 'u_pol_x', 'u_pol_wz',
            'v_cmd_in_x', 'v_cmd_in_wz', 'v_guard_x', 'v_guard_wz', 'rtf_10s', 'proj_grav_z')
    trace = {k: np.array([float(r[k]) for r in rows]) for k in keys}
    trace['state'] = [r['state'] for r in rows]
    trace['cmd_status'] = [r['cmd_status'] for r in rows]
    return trace


def settle_analysis(trace, goal, t_from, tol, settle_s, window=.5, v_max=.05, w_max=.05):
    """Earliest t>=t_from from which, for settle_s, the robot stays inside the
    position/yaw tolerance and the W05 0.5 s window speed/yaw-rate criterion."""
    t = trace['sim_time']
    yaw = np.unwrap(trace['yaw_w'])
    dt = t[1]-t[0]
    lag = int(round(window/dt))
    n = len(t)
    pos_err = np.hypot(trace['x_w']-goal[0], trace['y_w']-goal[1])
    yaw_err = np.abs([angle(a-goal[2]) for a in trace['yaw_w']])
    speed = np.full(n, np.inf)
    rate = np.full(n, np.inf)
    speed[lag:] = np.hypot(trace['x_w'][lag:]-trace['x_w'][:-lag], trace['y_w'][lag:]-trace['y_w'][:-lag])/window
    rate[lag:] = np.abs(yaw[lag:]-yaw[:-lag])/window
    ok = (pos_err <= tol['position_m']) & (yaw_err <= math.radians(tol['yaw_deg'])) & (speed < v_max) & (rate < w_max)
    hold = int(round(settle_s/dt))
    start = int(np.searchsorted(t, t_from))
    for i in range(start, n-hold):
        if ok[i:i+hold+1].all():
            j = i + hold
            return dict(settled=True, settle_start_sim=float(t[i]), settle_delay_s=float(t[i]-t_from),
                        pos_err_m=float(pos_err[j]), yaw_err_deg=float(math.degrees(yaw_err[j])))
    last = n-1
    return dict(settled=False, pos_err_m=float(pos_err[last]), yaw_err_deg=float(math.degrees(yaw_err[last])),
                final_window_speed=float(speed[last]), final_window_rate=float(rate[last]))


def log_counts(path, after_marker='navigation active'):
    text = Path(path).read_text(errors='replace')
    index = text.find(after_marker)
    tail = text[index:] if index >= 0 else text
    return {k: len(re.findall(p, tail)) for k, p in LOG_PATTERNS.items()}, index >= 0


def check_parameters(run):
    nodes = {'controller_server': ['use_sim_time', 'enable_stamped_cmd_vel', 'controller_frequency'],
             'behavior_server': ['use_sim_time', 'enable_stamped_cmd_vel', 'behavior_plugins', 'robot_base_frame'],
             'planner_server': ['use_sim_time', 'planner_plugins'], 'bt_navigator': ['use_sim_time', 'robot_base_frame'],
             'velocity_smoother': ['use_sim_time', 'enable_stamped_cmd_vel', 'feedback'],
             'collision_monitor': ['use_sim_time', 'enable_stamped_cmd_vel', 'base_frame_id'],
             'amcl': ['use_sim_time', 'base_frame_id'], 'local_costmap/local_costmap': ['robot_base_frame', 'robot_radius', 'use_sim_time'],
             'global_costmap/global_costmap': ['robot_base_frame', 'robot_radius', 'use_sim_time'],
             't1_navigation_adapter': ['use_sim_time'], 't1_command_gate': ['use_sim_time', 'command_source']}
    values = {}
    for node, names in nodes.items():
        response = run.call(GetParameters, f'/{node}/get_parameters', GetParameters.Request(names=names))
        values[node] = {}
        for name, v in zip(names, response.values):
            values[node][name] = [None, v.bool_value, v.integer_value, v.double_value, v.string_value, None, None, None, None,
                                  list(v.string_array_value)][v.type] if v.type in (1, 2, 3, 4, 9) else None
    checks = dict(sim_time_everywhere=all(v.get('use_sim_time') is True for k, v in values.items() if 'use_sim_time' in v),
                  unstamped=all(v.get('enable_stamped_cmd_vel') is False for v in values.values() if 'enable_stamped_cmd_vel' in v),
                  base_footprint=all(v.get(k) == 'base_footprint' for v in values.values() for k in ('robot_base_frame', 'base_frame_id') if k in v),
                  wait_only_behaviors=values['behavior_server']['behavior_plugins'] == ['wait'],
                  nav_radius=values['local_costmap/local_costmap']['robot_radius'] == 2.05 == values['global_costmap/global_costmap']['robot_radius'],
                  gate_navigation=values['t1_command_gate']['command_source'] == 'navigation')
    return values, checks


def graph_checks(closed):
    g = closed['graph']
    others = lambda topic: [n for n in g[topic]['subscribers'] if n != 'w12_evaluator']  # recorder excluded
    raw_pubs = set(g['/t1/nav/cmd_vel_raw']['publishers'])
    return dict(final_only_collision_monitor=g['/cmd_vel']['publishers'] == ['collision_monitor'],
                gate_only_cmd_vel_nav=g['/cmd_vel_nav']['publishers'] == ['t1_command_gate'],
                adapter_only_navigation_source=g['/t1/cmd_navigation']['publishers'] == ['t1_navigation_adapter'],
                nav2_into_adapter_only=raw_pubs <= {'controller_server', 'behavior_server'} and
                others('/t1/nav/cmd_vel_raw') == ['t1_navigation_adapter'],
                no_teleop_source=not g['/t1/cmd_teleop']['publishers'] and not others('/t1/cmd_teleop'),
                one_clock=g['/clock']['publishers'] == ['t1_sim'],
                truth_isolated=set(g['/t1/eval/ground_truth']['subscribers']) <= {'w12_evaluator'},
                debug_disabled=not g['/t1/debug/policy_cmd']['subscribers'],
                twist_only=all(g[t]['types'] in ([], ['geometry_msgs/msg/Twist']) for t in ('/cmd_vel', '/cmd_vel_nav', '/cmd_vel_smoothed', '/t1/cmd_navigation')),
                amcl_map_odom=closed['tf_owners'].get('odom') == ['amcl'],
                sim_base=closed['tf_owners'].get('base_footprint') == ['t1_sim'],
                unique_tf=all(len(v) == 1 for v in closed['tf_owners'].values()))


def write_layers(run, name):
    path = OUT/f'{name}-layers.csv'
    keys = ['sim', 'tx', 'ty', 'tyaw', 'mx', 'my', 'myaw', 'vx', 'wz', 'gate', 'cm', 'cmd_status', 'adapter_reason']
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(keys + [f'{k}_{a}' for k, _ in LAYERS for a in ('vx', 'wz')] + ['u_pol_x', 'u_pol_wz', 'v_guard_x', 'v_guard_wz'])
        for r in run.node.rows:
            line = [r[k] if not isinstance(r[k], float) else round(r[k], 4) for k in keys]
            for k, _ in LAYERS:
                line += [round(v, 4) for v in r[k]] if r[k] else ['', '']
            line += [round(r['u_pol'][0], 4), round(r['u_pol'][2], 4)] if r['u_pol'] else ['', '']
            line += [round(r['v_guard'][0], 4), round(r['v_guard'][2], 4)] if r['v_guard'] else ['', '']
            w.writerow(line)
    return path


def layer_summary(rows):
    reasons = {}
    for r in rows:
        if r['adapter_reason']:
            reasons[r['adapter_reason']] = reasons.get(r['adapter_reason'], 0) + 1
    moving = [r for r in rows if r['final'] and r['final'][0] > 0]
    ratio = [r['vx']/r['final'][0] for r in moving if r['final'][0] >= .315]
    return dict(adapter_reasons=reasons, samples=len(rows),
                final_vs_measured_vx_ratio_median=float(np.median(ratio)) if ratio else None,
                adapter_differs_from_raw=sum(1 for r in rows if r['nav_raw'] and r['adapter'] and
                                             (abs(r['nav_raw'][0]-r['adapter'][0]) > 1e-6 or abs(r['nav_raw'][1]-r['adapter'][1]) > 1e-6)),
                final_less_than_gate=sum(1 for r in rows if r['gate'] and r['final'] and r['final'][0] < r['gate'][0]-1e-6))


def dynamic_evidence(run, result, trace):
    """Evaluation only: correlate scan/costmap/commands with kinetic obstacle."""
    label = result['label']
    obstacle = run.node.obstacles
    scans, costs = run.node.dynamic_scans, run.node.cost_samples
    trigger = next((o['triggered_at'] for o in obstacle if o['triggered_at'] is not None), None)
    hit_frames = [s for s in scans if s['hits'] and s['obstacle']['triggered_at'] is not None]
    marked = [c for c in costs if c['max_cost'] == 100 and c['obstacle']['triggered_at'] is not None]
    cleared = [c for c in costs if c['obstacle']['triggered_at'] is not None
               and c['obstacle']['position'][1] >= 2.99
               and c['obstacle']['phase'].startswith('hold')
               and c['vacated_center_value'] == 0]
    kind = os.environ.get('W12_SCENARIO', '')
    t = trace['sim_time']
    snapshot_t = np.array([o['sim_time'] for o in obstacle])
    ox = np.interp(t, snapshot_t, [o['position'][0] for o in obstacle])
    oy = np.interp(t, snapshot_t, [o['position'][1] for o in obstacle])
    sx, sy, _ = obstacle[0]['size']
    distances = np.hypot(np.maximum(np.abs(trace['x_w']-ox)-sx/2, 0),
                         np.maximum(np.abs(trace['y_w']-oy)-sy/2, 0))
    window = 25  # W05 .5 s at 50 Hz
    speed = np.full(len(t), np.inf)
    speed[window:] = np.hypot(trace['x_w'][window:]-trace['x_w'][:-window],
                              trace['y_w'][window:]-trace['y_w'][:-window])/.5
    rate = np.full(len(t), np.inf)
    yaws = np.unwrap(trace['yaw_w'])
    rate[window:] = abs(yaws[window:]-yaws[:-window])/.5
    physical_stop = ((t > (trigger or 1e9)) & (speed < .05) & (rate < .05)
                     & (trace['u_pol_x'] == 0) & (trace['u_pol_wz'] == 0))
    protected = distances < 1.96
    stops = np.flatnonzero(physical_stop & protected)
    stop = float(t[stops[0]]) if len(stops) else None
    resumes = np.flatnonzero((t > (stop if stop is not None else 1e9)) & (speed > .2)
                             & (trace['u_pol_x'] > 0) & (distances > 1.95))
    resume = float(t[resumes[0]]) if len(resumes) else None
    local, poly = footprint_points()
    footprint_gaps = []
    for i in range(0, len(t), 5):
        c, sn = math.cos(trace['yaw_w'][i]), math.sin(trace['yaw_w'][i])
        vertices = local @ np.array([[c, -sn], [sn, c]]).T + [trace['x_w'][i], trace['y_w'][i]]
        rectangle = (ox[i]-sx/2, ox[i]+sx/2, oy[i]-sy/2, oy[i]+sy/2)
        footprint_gaps.append(float(rect_distance(vertices, rectangle).min()))
    stopped_hold_s = 0.
    streak = 0
    for active in physical_stop & protected:
        streak = streak + 1 if active else 0
        stopped_hold_s = max(stopped_hold_s, streak*.02)
    plans = run.node.plans
    initial_lateral = max((abs(p[1]) for p in plans[0]['points']), default=0) if plans else None
    after_plans = [p for p in plans if trigger and p['sim'] > trigger]
    replanned_lateral = max((abs(pt[1]) for p in after_plans for pt in p['points']), default=0)
    motion = t > (trigger if trigger is not None else 1e9)
    max_lateral = float(np.max(abs(trace['y_w'][motion]))) if motion.any() else 0.
    current = result['run']['graph']
    truth_subscribers = current['/t1/eval/obstacle']['subscribers']
    checks = dict(triggered_after_actual_walking=trigger is not None and bool((trace['u_pol_x'][t < trigger] > 0).any()),
                  scan_hits_entity=len(hit_frames) >= 3, local_costmap_marks_entity=len(marked) >= 2,
                  no_planner_obstacle_truth=truth_subscribers == ['w12_evaluator'],
                  no_teleop=current['/t1/cmd_teleop']['publishers'] == [],
                  dimensions=result['run']['manifest'].get('model_dimensions') == dict(nq=30,nv=29,nu=23,nmocap=1))
    if kind == 'blocking':
        checks.update(real_detour=max_lateral > 2.25 and replanned_lateral > 2.25,
                      blocker_never_removed=all(abs(o['position'][1]) < .001 for o in obstacle
                          if o['phase'].startswith('hold')),
                      goal_reached_while_blocked=obstacle[-1]['phase'] == 'hold_0' and abs(obstacle[-1]['position'][1]) < .001)
    else:
        checks['vacated_costmap_clears'] = len(cleared) >= 2
    if kind == 'approach':
        checks.update(actual_stop_in_protection=stop is not None and stopped_hold_s >= .5,
                      safely_resumed=resume is not None)
    checks['navigation_success'] = result['outcome'] == 'SUCCESS'
    header = 'sim_time,obstacle_x,obstacle_y,max_cost,vacated_cost,cmd_vel_x,cmd_vel_wz,plans'
    data = [[c['sim'], *c['obstacle']['position'][:2], c['max_cost'] if c['max_cost'] is not None else -999,
             c['vacated_center_value'] if c['vacated_center_value'] is not None else -999,
             *(c['final_cmd'] or [0,0]), c['plan_count']] for c in costs]
    np.savetxt(OUT/f'{label}-costmap.csv', data, delimiter=',', header=header, comments='', fmt='%.4f')
    np.savetxt(OUT/f'{label}-obstacle.csv', [[o['sim_time'], *o['position'], o['triggered_at'] or -1] for o in obstacle],
               delimiter=',', header='sim_time,x,y,z,triggered_at', comments='', fmt='%.4f')
    samples = [s for i, s in enumerate(hit_frames) if i in (0,len(hit_frames)//2,len(hit_frames)-1)]
    for s in samples:
        s['hits'] = s['hits'][:12]
    result['dynamic'] = dict(scenario=kind, checks=checks, verdict='PASS' if all(checks.values()) else 'FAIL',
                             trigger_sim=trigger, scan_hit_frames=len(hit_frames), costmap_marked_samples=len(marked),
                             costmap_cleared_samples=len(cleared), actual_stop_sim=stop, resume_sim=resume,
                             stopped_hold_s=stopped_hold_s, min_center_surface_gap_m=float(distances.min()),
                             min_rotated_footprint_gap_m=min(footprint_gaps),
                             max_robot_lateral_m=max_lateral, initial_plan_lateral_m=initial_lateral,
                             replanned_lateral_m=replanned_lateral, scan_topic_samples=samples)
    (RAW/f'{label}-topic-data.json').write_text(json.dumps(dict(obstacles=obstacle, scans=scans, costs=costs, plans=plans)))
    if not all(checks.values()):
        result['outcome'] = 'FAIL_DYNAMIC'
    return result['dynamic']


def episode(label, start, goal, tol, time_limit, rviz=False, case_id=None, seed=0, screenshot=None, post_s=12., tool_args=()):
    run = Run('navigation.launch.py', label, [
        f'scene:={os.environ.get("W12_SCENE", "dynamic_crossing_world")}', f'start_pose:={start[0]} {start[1]} {start[2]}',
        f'initial_pose:={start[0]} {start[1]} {start[2]}', f'rviz:={str(rviz).lower()}',
        f'max_sim_time:={time_limit + 60:.0f}', 'headless:=true', f'seed:={seed}',
        *([f'localization_params_file:={WS}/src/t1_navigation/config/w12_localization.yaml']
          if os.environ.get('W12_SCENE','dynamic_crossing_world') == 'dynamic_crossing_world' else []), f'map:={META.with_suffix("").with_suffix(".yaml")}',
        *([f'events_file:={WS}/src/t1_bringup/config/w12_events.yaml', f'scenario:={os.environ["W12_SCENARIO"]}']
          if os.environ.get('W12_SCENARIO') else [])], wall_limit=time_limit*2.5 + 200)
    result = dict(label=label, case=case_id, seed=seed, start=start, goal=goal, tolerances=tol, time_limit_s=time_limit)
    tool = None
    try:
        run.ready()
        run.until(lambda: any(e['kind'] == 'gate' for e in run.node.events), 30)
        # No lifecycle service polling during bringup (it raced AMCL/bt_navigator
        # transitions); the starter prints this line after STARTUP succeeded.
        nav_active = lambda: '"starter": "navigation active"' in run.log_path.read_text(errors='replace')
        run.until(nav_active, 120)
        result['nav_active_sim'] = run.node.sim
        params, param_ok = check_parameters(run)
        result.update(parameters=params, parameter_checks=param_ok)
        tool = run.start_child(['ros2', 'run', 't1_bringup', 'navigate_to_pose', *map(str, goal),
                                '--time-limit', str(time_limit), *tool_args], 'goal')
        shot_taken = False
        while tool.poll() is None:
            run.spin()
            if screenshot and rviz and not shot_taken and run.node.plans and run.node.sim > result['nav_active_sim'] + screenshot:
                run.screenshot(OUT/f'{label}-midroute.png')
                shot_taken = True
        result['tool_exit'] = tool.returncode
        end = run.node.sim + post_s
        run.until(lambda: run.node.sim >= end, post_s*3 + 30)
        if rviz:
            run.screenshot(OUT/f'{label}-final.png')
    except Exception as exc:
        result['error'] = repr(exc)
        print('ERROR', label, repr(exc), flush=True)
    closed = run.close(settle=0.)
    result['run'] = closed
    goal_log = RAW/f"{run.run_id}-goal.txt"
    lines = [json.loads(l) for l in goal_log.read_text().splitlines() if l.startswith('{')] if goal_log.exists() else []
    nav = next((l for l in lines if l.get('event') == 'result'), {})
    result['nav2'] = nav
    trace = load_trace(run.run_id)
    meta = yaml.safe_load(META.read_text())
    T = meta['T_world_map']
    # Truth evaluation in the map frame (T_world_map is identity up to 1e-18 here; applied anyway).
    c, s = math.cos(T['yaw']), math.sin(T['yaw'])
    gx, gy = T['x'] + c*goal[0] - s*goal[1], T['y'] + s*goal[0] + c*goal[1]
    goal_world = (gx, gy, goal[2] + T['yaw'])
    t_success = nav.get('nav2_success_sim') if nav.get('status') == 'SUCCEEDED' else None
    settle = settle_analysis(trace, goal_world, t_success if t_success else trace['sim_time'][-1], tol, tol['settle_s']) \
        if t_success else dict(settled=False)
    center_clear, foot_clear = clearances(trace, scene_rectangles())
    counts, activated = log_counts(run.log_path)
    contact = closed['manifest'].get('contact_counts') or {}
    fallen = any(s in ('FALLEN', 'FAULT') for s in trace['state'])
    path_len = float(np.sum(np.hypot(np.diff(trace['x_w']), np.diff(trace['y_w']))))
    straight = math.dist(start[:2], goal[:2])
    plans = run.node.plans
    events = closed['events']
    safety = dict(cm_stop=sum(1 for e in events if e['kind'] == 'collision_monitor' and e['action_type'] == 1 and e['sim'] > 1.),
                  cm_slowdown=sum(1 for e in events if e['kind'] == 'collision_monitor' and e['action_type'] == 2),
                  gate_stopped=[e for e in events if e['kind'] == 'gate' and e['state'] == 'STOPPED'],
                  bridge_timeouts=sum(1 for i, st in enumerate(trace['cmd_status']) if st in ('TIMEOUT', 'INVALID') and trace['state'][i] == 'READY'))
    first_plan = plans[0] if plans else None
    detour = None
    if first_plan:
        pts = np.array(first_plan['points'])
        a, b = np.array(start[:2]), np.array(goal[:2])
        d = b-a
        lateral = np.abs(d[0]*(pts[:, 1]-a[1]) - d[1]*(pts[:, 0]-a[0]))/np.linalg.norm(d)
        detour = dict(first_plan_length_m=first_plan['length'], straight_m=straight,
                      max_lateral_from_straight_m=float(lateral.max()))
    metrics = dict(
        nav2_status=nav.get('status'), nav2_error_code=nav.get('error_code'), tool_exit=result.get('tool_exit'),
        time_to_goal_s=(t_success - nav['accepted_sim']) if t_success else None, task_duration_s=nav.get('duration_sim'),
        terminal_retries=nav.get('terminal_retries'), attempts=nav.get('attempts'), verified=nav.get('verified'),
        settle=settle, arrived_truth=bool(settle.get('settled')), path_len_m=path_len, straight_m=straight, detour=detour,
        n_plans=len(plans), clear_center_min_m=center_clear, clear_footprint_min_m=foot_clear,
        scan_min_range_m=closed['scan_stats'].get('min_range'), env_contacts=contact, fell=fallen,
        safety=dict(safety, gate_stopped=len(safety['gate_stopped'])), log_counts=counts, nav_activated_in_log=activated,
        scan_empty=closed['scan_stats']['empty'], scan_known_min=closed['scan_stats']['known_min'],
        rtf_median=float(np.median(trace['rtf_10s'][trace['rtf_10s'] > 0])) if (trace['rtf_10s'] > 0).any() else None,
        sim_time_s=float(trace['sim_time'][-1]), wall_time_s=closed['wall_s'], min_trunk_z=float(trace['z_w'].min()))
    metrics['graph_checks'] = graph_checks(closed)
    result['metrics'] = metrics
    result['layers_csv'] = str(write_layers(run, label).relative_to(WS))
    result['layer_summary'] = layer_summary(run.node.rows)
    result['plans'] = plans[:1] + plans[-1:]
    np.savetxt(OUT/f'{label}-trajectory.csv', np.c_[trace['sim_time'], trace['x_w'], trace['y_w'], trace['yaw_w'],
               trace['v_meas_x'], trace['v_meas_wz'], trace['v_cmd_in_x'], trace['v_cmd_in_wz'], trace['u_pol_x'], trace['u_pol_wz']][::5],
               delimiter=',', header='sim_time,x_w,y_w,yaw_w,v_meas_x,v_meas_wz,cmd_vel_x,cmd_vel_wz,u_pol_x,u_pol_wz', comments='', fmt='%.4f')
    success = (metrics['nav2_status'] == 'SUCCEEDED' and bool(metrics['verified']) and result.get('tool_exit') == 0 and metrics['arrived_truth'] and not metrics['fell'] and
               not contact.get('static_env') and not contact.get('obstacle') and not contact.get('ground_nonfoot') and
               (metrics['time_to_goal_s'] or 1e9) <= time_limit)
    success = (success and closed['exit_code'] == 0 and closed['manifest'].get('exit_code') == 0
               and set(contact) == {'static_env','obstacle','ground_nonfoot'})
    success = success and all(metrics['graph_checks'].values()) and all(result.get('parameter_checks', {}).values())
    result['outcome'] = 'SUCCESS' if success else (
        'FAIL_FALL' if metrics['fell'] else 'FAIL_COLLISION' if contact.get('static_env') else
        'FAIL_TIMEOUT' if nav.get('timed_out') else 'FAIL_PLANNING' if metrics['nav2_status'] == 'ABORTED' else 'FAIL')
    if os.environ.get('W12_SCENARIO') and run.node.obstacles:
        dynamic_evidence(run, result, trace)
    (OUT/f'{label}.json').write_text(json.dumps(result, indent=2, default=str) + '\n')
    print('EPISODE', label, result['outcome'], json.dumps({k: metrics[k] for k in (
        'nav2_status', 'time_to_goal_s', 'arrived_truth', 'path_len_m', 'clear_center_min_m', 'clear_footprint_min_m',
        'env_contacts', 'fell', 'rtf_median')}), json.dumps(settle), flush=True)
    return result


def cancel_replace(cases):
    """F09 with a real NavigateToPose: cancel mid-route -> stop service latch;
    explicit arm; new goal; preempt it with a replacement goal."""
    spec = cases['cancel_replace']
    start, g1, g2, g3 = spec['start'], spec['first_goal'], spec['second_goal'], spec['replacement_goal']
    label = 'w12_cancel_replace'
    run = Run('navigation.launch.py', 'cancel', [f'scene:={os.environ.get("W12_SCENE", "dynamic_crossing_world")}', f'start_pose:={start[0]} {start[1]} {start[2]}',
              f'initial_pose:={start[0]} {start[1]} {start[2]}', 'rviz:=false', 'max_sim_time:=240'], wall_limit=600)
    result = dict(spec=spec)
    client = ActionClient(run.node, NavigateToPose, 'navigate_to_pose')

    def send(goal):
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal[0], goal[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(goal[2]/2), math.cos(goal[2]/2)
        future = client.send_goal_async(msg)
        run.until(future.done, 15)
        handle = future.result()
        assert handle.accepted
        return handle

    def arm():
        run.until(lambda: run.node.gate and not run.node.gate['health_reason'] and run.node.gate['state'] != 'ARMED'
                  or run.node.gate['state'] == 'ARMED', 30)
        for _ in range(40):
            if run.node.gate['state'] == 'ARMED':
                return
            r = run.call(Trigger, '/t1/safety/arm', Trigger.Request())
            if r.success:
                run.until(lambda: run.node.gate['state'] == 'ARMED', 5)
                return
            run.duration(.3)
        raise AssertionError('arm failed')
    try:
        run.ready()
        run.until(lambda: '"starter": "navigation active"' in run.log_path.read_text(errors='replace'), 120)
        run.until(lambda: client.server_is_ready(), 30)
        arm()
        h1 = send(g1)
        t_accept = run.node.sim
        run.until(lambda: run.node.rows and run.node.rows[-1]['vx'] > .2, 60)
        run.duration(spec['cancel_after_moving_s'])
        moving = run.node.rows[-1]
        t_cancel, wall_cancel = run.node.sim, time.monotonic()
        cancel = h1.cancel_goal_async()
        run.until(cancel.done, 10)
        r1 = h1.get_result_async()
        run.until(r1.done, 15)
        run.until(lambda: run.node.gate['state'] == 'STOPPED', 5)
        t_stopped = run.node.sim
        run.duration(6.)
        result['cancel'] = dict(first_goal_status=r1.result().status, t_accept=t_accept, t_cancel=t_cancel,
                                gate_stopped_after_s=t_stopped-t_cancel, gate_reason=run.node.gate['reason'],
                                v_at_cancel=moving['vx'], pose_at_cancel=[moving['tx'], moving['ty']],
                                pose_after_6s=[run.node.rows[-1]['tx'], run.node.rows[-1]['ty']])
        arm()
        h2 = send(g2)
        run.until(lambda: run.node.rows[-1]['vx'] > .2, 60)
        run.duration(spec['replace_after_moving_s'])
        t_replace = run.node.sim
        h3 = send(g3)
        r2 = h2.get_result_async()
        r3 = h3.get_result_async()
        run.until(r3.done, 200)
        result['replace'] = dict(t_replace=t_replace, second_goal_status=r2.result().status if r2.done() else None,
                                 replacement_status=r3.result().status, t_done=run.node.sim)
        run.duration(6.)
    except Exception as exc:
        result['error'] = repr(exc)
        print('ERROR cancel', repr(exc), flush=True)
    closed = run.close(settle=0.)
    result['run'] = closed
    trace = load_trace(run.run_id)
    t = trace['sim_time']
    if 'cancel' in result:
        c = result['cancel']
        after = t > c['t_cancel']
        zero = np.flatnonzero(after & (trace['u_pol_x'] == 0) & (trace['u_pol_wz'] == 0))
        t_zero = float(t[zero[0]]) if len(zero) else None
        i0 = int(np.searchsorted(t, c['t_cancel']))
        iz = int(np.searchsorted(t, t_zero)) if t_zero else len(t)-1
        stop_disp = np.hypot(trace['x_w'][iz:]-trace['x_w'][iz], trace['y_w'][iz:]-trace['y_w'][iz])
        restart = np.flatnonzero((t > t_zero) & (np.abs(trace['u_pol_x']) + np.abs(trace['u_pol_wz']) > 0)) if t_zero else []
        c.update(t_policy_zero=t_zero, cancel_to_policy_zero_s=(t_zero - c['t_cancel']) if t_zero else None,
                 cancel_to_zero_disp_m=float(math.hypot(trace['x_w'][iz]-trace['x_w'][i0], trace['y_w'][iz]-trace['y_w'][i0])),
                 max_disp_after_zero_m=float(stop_disp[:int(6/.02)].max()),
                 first_nonzero_after_cancel_sim=float(t[restart[0]]) if len(restart) else None)
    if 'replace' in result:
        r = result['replace']
        final = (float(trace['x_w'][-1]), float(trace['y_w'][-1]))
        r.update(final_truth=final, dist_to_replacement=math.dist(final, g3[:2]), dist_to_second=math.dist(final, g2[:2]))
    checks = {}
    if 'cancel' in result and 'replace' in result:
        c, r = result['cancel'], result['replace']
        checks = dict(first_goal_canceled=c['first_goal_status'] == 5, gate_latched_on_cancel=c['gate_reason'] == 'EXPLICIT_STOP_OR_CANCEL',
                      policy_zero_within_permit_budget=c['cancel_to_policy_zero_s'] is not None and c['cancel_to_policy_zero_s'] <= .5,
                      stop_within_w10_envelope=c['max_disp_after_zero_m'] <= .453,
                      no_motion_until_rearmed_new_goal=c['first_nonzero_after_cancel_sim'] is None or c['first_nonzero_after_cancel_sim'] > c['t_cancel'] + 6.,
                      replaced_goal_not_succeeded=r['second_goal_status'] != 4,
                      replacement_succeeded=r['replacement_status'] == 4,
                      ended_at_replacement=r['dist_to_replacement'] <= .30,
                      no_env_contact=not (closed['manifest'].get('contact_counts') or {}).get('static_env'),
                      clean_exit=closed['exit_code'] == 0)
        checks.update(graph_checks(closed))
    result['checks'] = checks
    result['verdict'] = 'PASS' if checks and all(checks.values()) else 'FAIL'
    result['layers_csv'] = str(write_layers(run, label).relative_to(WS))
    np.savetxt(OUT/f'{label}-trajectory.csv', np.c_[t, trace['x_w'], trace['y_w'], trace['yaw_w'], trace['v_meas_x'],
               trace['u_pol_x'], trace['u_pol_wz']][::5], delimiter=',', header='sim_time,x_w,y_w,yaw_w,v_meas_x,u_pol_x,u_pol_wz',
               comments='', fmt='%.4f')
    (OUT/f'{label}.json').write_text(json.dumps(result, indent=2, default=str) + '\n')
    print('CANCEL_REPLACE', result['verdict'], json.dumps(checks), json.dumps({k: result.get(k) for k in ('cancel', 'replace')}, default=str), flush=True)
    return result


def startup_probe(n):
    """Startup reliability: n fresh navigation launches (RViz on, worst case observed),
    each counted only if the readiness starter reports navigation active."""
    rows = []
    for i in range(n):
        run = Run('navigation.launch.py', f'startup{i}', [f'scene:={os.environ.get("W12_SCENE", "dynamic_crossing_world")}', 'start_pose:=0 0 0',
                  'initial_pose:=0 0 0', 'rviz:=true', 'max_sim_time:=60'], wall_limit=150, authority=False)
        t0, ok, error = time.monotonic(), False, None
        try:
            run.ready()
            run.until(lambda: '"starter": "navigation active"' in run.log_path.read_text(errors='replace'), 90)
            ok = True
        except Exception as exc:
            error = repr(exc)
        text = run.log_path.read_text(errors='replace')
        rows.append(dict(i=i, run_id=run.run_id, navigation_active=ok, wall_to_active_s=time.monotonic()-t0 if ok else None,
                         sim_at_active=run.node.sim if ok else None, error=error,
                         lost_responses=text.count('failed to send response'), bond_failures=text.count('CRITICAL FAILURE')))
        run.close(settle=0.)
        print('STARTUP', rows[-1], flush=True)
    result = dict(code_sha=git('rev-parse', 'HEAD'), runs=rows, active=sum(r['navigation_active'] for r in rows), n=n)
    (OUT/'c3-startup-reliability.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['dev', 'case', 'suite', 'cancel', 'startup'])
    parser.add_argument('case', nargs='?')
    parser.add_argument('--start')
    parser.add_argument('--goal')
    parser.add_argument('--label', default='dev')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--time-limit', type=float, default=120.)
    parser.add_argument('--tool-args', default='', help='extra navigate_to_pose options (dev only)')
    opts = parser.parse_args()
    cases = yaml.safe_load(CASES.read_text()) if opts.mode not in ('dev', 'startup') else {}
    session = Session('nav_' + opts.mode)
    results = []
    try:
        if opts.mode == 'dev':
            tol = dict(position_m=.30, yaw_deg=20., settle_s=2.)
            results.append(episode(f'w12_dev_{opts.label}', [float(v) for v in opts.start.split()],
                                   [float(v) for v in opts.goal.split()], tol, opts.time_limit, rviz=opts.rviz,
                                   tool_args=opts.tool_args.split()))
        elif opts.mode in ('case', 'suite'):
            frozen = cases['criteria']
            selected = [c for c in cases['cases'] if opts.mode == 'suite' or c['id'] == opts.case]
            assert selected, 'unknown case'
            for c in selected:
                results.append(episode(f"w12_{c['id']}", c['start'], c['goal'], frozen, frozen['time_limit_s'],
                                       rviz=c.get('rviz', False), case_id=c['id'], seed=c['seed'], screenshot=c.get('screenshot_after_s')))
        elif opts.mode == 'startup':
            startup_probe(int(opts.case or 8))
        else:
            results.append(cancel_replace(cases))
    finally:
        session.close()
    if opts.mode == 'suite':
        summary = dict(code_sha=git('rev-parse', 'HEAD'), cases_sha256=digest(CASES),
                       results=[dict(case=r['case'], outcome=r['outcome'], **{k: r['metrics'][k] for k in (
                           'nav2_status', 'time_to_goal_s', 'arrived_truth', 'path_len_m', 'clear_center_min_m',
                           'clear_footprint_min_m', 'env_contacts', 'fell', 'rtf_median')}, settle=r['metrics']['settle'])
                                for r in results])
        summary['verdict'] = 'PASS' if len(results) == len(cases['cases']) and all(r['outcome'] == 'SUCCESS' for r in results) else 'FAIL'
        (OUT/'suite.json').write_text(json.dumps(summary, indent=2) + '\n')
        print('SUITE', summary['verdict'], flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

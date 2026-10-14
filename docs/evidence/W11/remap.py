"""ADR-015: rebuild the static_world v2 map with SLAM Toolbox from real walking.

Same method as W09 (predetermined inspection route replayed with odom-based
heading/stop decisions; no planner, no geometry rasterization) but through the
W10 chain: /t1/cmd_teleop -> gate (explicit arm) -> smoother -> monitor -> bridge.
Every waypoint and leg keeps >=2.25 m to all surfaces (scene_feasibility.py), so
in-place turns stay outside the 1.95 m slowdown circle.
"""
import json
import math
import subprocess

import numpy as np
from slam_toolbox.srv import SerializePoseGraph

from harness import OUT, RAW, WS, Run, Session, angle, yaw

ROUTE = [(1.5, 2.75), (7.5, 2.75), (9.0, 0.0), (7.5, -2.75), (1.5, -2.75), (0.0, 0.0)]


def drive(run):
    legs = []
    for i, target in enumerate(ROUTE):
        start = run.node.sim
        p = run.node.odom.pose.pose.position
        heading = math.atan2(target[1]-p.y, target[0]-p.x)
        error = lambda: angle(heading - yaw(run.node.odom.pose.pose.orientation))
        if abs(error()) > .16:
            run.source = (0., .6 if error() > 0 else -.6)
            run.until(lambda: abs(error()) < .16, 25)
            run.source = (0., 0.)
            run.duration(2.5)
        while True:
            p = run.node.odom.pose.pose.position
            if math.hypot(target[0]-p.x, target[1]-p.y) < .23:
                break
            desired = math.atan2(target[1]-p.y, target[0]-p.x)
            e = angle(desired - yaw(run.node.odom.pose.pose.orientation))
            run.source = (.45, max(-.4, min(.4, 1.6*e)))
            run.spin()
            assert run.node.gate['state'] == 'ARMED', f'gate latched: {run.node.gate}'
        run.source = (0., 0.)
        run.duration(2.5)
        legs.append(dict(leg=i, target=target, sim_start=start, sim_end=run.node.sim,
                         actual=[run.node.rows[-1][k] for k in ('x', 'y', 'yaw')]))
        print('ARRIVED', i, legs[-1]['actual'], flush=True)
        if i in (1, 3):
            run.screenshot(OUT/f'mapping-leg-{i}.png')
    return legs


def main():
    session = Session('remap')
    result, run = dict(route=ROUTE), None
    try:
        run = Run('mapping.launch.py', 'mapping', ['scene:=static_world', 'rviz:=true', 'max_sim_time:=300.0'])
        run.ready()
        run.until(lambda: run.node.map is not None and run.node.map_tf is not None, 60)
        t = run.node.map_tf.transform
        result.update(initial_map_odom=[t.translation.x, t.translation.y, yaw(t.rotation)],
                      initial_odom=[run.node.rows[-1][k] for k in ('x', 'y', 'yaw')])
        run.arm_teleop()
        result['legs'] = drive(run)
        run.duration(3.)
        run.screenshot(OUT/'mapping-final.png')
        m = run.node.map
        np.savez_compressed(RAW/'slam-grid.npz', grid=np.array(m.data).reshape(m.info.height, m.info.width),
                            resolution=m.info.resolution,
                            origin=[m.info.origin.position.x, m.info.origin.position.y, yaw(m.info.origin.orientation)])
        t = run.node.map_tf.transform
        result['final_map_odom'] = [t.translation.x, t.translation.y, yaw(t.rotation)]
        base = WS/'src/t1_navigation/maps/static_world'
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(base), '--ros-args',
               '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=10.0']
        saver = subprocess.Popen(cmd, stdout=(RAW/'map-save.txt').open('w'), stderr=subprocess.STDOUT)
        run.until(lambda: saver.poll() is not None, 30)
        assert saver.returncode == 0, 'map save failed'
        response = run.call(SerializePoseGraph, '/slam_toolbox/serialize_map',
                            SerializePoseGraph.Request(filename=str(RAW/'static_world')))
        assert response.result == 0, 'pose graph serialization failed'
        result['map_save_command'] = cmd
        result['map_updates'] = run.node.map_updates
        result['gate_never_latched'] = not any(e['kind'] == 'gate' and e['state'] == 'STOPPED' for e in run.node.events)
        result['cm_actions'] = sorted({e['action_type'] for e in run.node.events if e['kind'] == 'collision_monitor'})
        result['verdict'] = 'PASS'
    except Exception as exc:
        result.update(verdict='FAIL', error=repr(exc))
        print('ERROR', repr(exc), flush=True)
    finally:
        if run is not None:
            closed = run.close()
            rows = run.node.rows
            np.savetxt(OUT/(run.run_id + '-trajectory.csv'),
                       np.array([[r['sim'], r['x'], r['y'], r['yaw'], r['vx'], r['wz']] for r in rows]),
                       delimiter=',', header='sim_time,x,y,yaw,vx,wz', comments='', fmt='%.5f')
            result['run'] = closed
            checks = dict(slam_only_map_odom=closed['tf_owners'].get('odom') == ['slam_toolbox'],
                          sim_owns_base=closed['tf_owners'].get('base_footprint') == ['t1_sim'],
                          clean_exit=closed['exit_code'] == 0 and closed['manifest'].get('exit_code') == 0,
                          no_clock_jump=closed['clock_regressions'] == 0,
                          no_env_contact=(closed['manifest'].get('contact_counts') or {}).get('static_env', 1) == 0)
            result['checks'] = checks
            if not all(checks.values()):
                result['verdict'] = 'FAIL'
        session.close()
        (OUT/'mapping.json').write_text(json.dumps(result, indent=2) + '\n')
    print('VERDICT', result['verdict'], result.get('checks'), flush=True)
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

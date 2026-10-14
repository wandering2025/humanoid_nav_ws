"""ADR-015: re-run the W09 three-start AMCL acceptance on the v2 map.

Same frozen limits and initial inputs (from map metadata), same evaluation as
docs/evidence/W09/check_localization.py; motion now goes through the W10 chain
(/t1/cmd_teleop + explicit arm). One approximate /initialpose per startup.
"""
import json
import math

import numpy as np
import yaml
from lifecycle_msgs.srv import GetState
from rclpy.time import Time

from harness import OUT, RAW, WS, Run, Session, digest, ns, yaw


def evaluate(run, initial_sim, meta):
    rows = np.array([[r['sim'], r['x'], r['y'], r['yaw']] for r in run.node.rows])
    poses = np.array(run.node.poses)
    assert len(poses) > 5, 'AMCL pose updates missing'
    transform = meta['T_world_map']
    a = transform['yaw']
    c, s = math.cos(a), math.sin(a)
    xy = poses[:, 1:3] @ np.array([[c, -s], [s, c]]).T + np.array([transform['x'], transform['y']])
    truth = np.c_[np.interp(poses[:, 0], rows[:, 0], rows[:, 1]), np.interp(poses[:, 0], rows[:, 0], rows[:, 2])]
    true_yaw = np.interp(poses[:, 0], rows[:, 0], np.unwrap(rows[:, 3]))
    pos = np.linalg.norm(xy - truth, axis=1)
    rotation = np.abs(np.arctan2(np.sin(poses[:, 3] + a - true_yaw), np.cos(poses[:, 3] + a - true_yaw)))
    tol = meta['expected_localization']
    ok = (pos <= tol['position_error_max_m']) & (rotation <= tol['yaw_error_max_rad'])
    convergence = next((float(poses[i, 0] - initial_sim) for i in range(len(poses)) if ok[i:].all()), None)
    np.savetxt(OUT/(run.run_id + '-pose-errors.csv'), np.c_[poses[:, 0], pos, rotation], delimiter=',',
               header='sim_time,position_error_m,yaw_error_rad', comments='', fmt='%.6f')
    grid = run.node.map
    data = np.array(grid.data).reshape(grid.info.height, grid.info.width)
    iy, ix = np.where(data >= 65)
    occupied = np.c_[ix + .5, iy + .5]*grid.info.resolution + np.array([grid.info.origin.position.x, grid.info.origin.position.y])
    distances, lookups, errors = [], 0, []
    for scan in run.node.scans[-8:]:
        try:
            tf = run.node.buffer.lookup_transform('map', 'laser_stabilized', Time(nanoseconds=ns(scan.header.stamp)))
        except Exception as exc:
            errors.append(str(exc))
            continue
        p, theta = tf.transform.translation, yaw(tf.transform.rotation)
        ranges = np.array(scan.ranges)
        valid = np.isfinite(ranges)
        angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment + theta
        points = np.c_[ranges[valid]*np.cos(angles[valid]) + p.x, ranges[valid]*np.sin(angles[valid]) + p.y]
        d = np.sqrt(np.min(np.sum((points[:, None, :] - occupied[None, :, :])**2, axis=2), axis=1))
        distances.extend(d)
        lookups += 1
    p95 = float(np.percentile(distances, 95)) if distances else None
    return dict(pose_updates=len(poses), position_error_final_m=float(pos[-1]), yaw_error_final_rad=float(rotation[-1]),
                yaw_error_final_deg=float(math.degrees(rotation[-1])), convergence_sim_s=convergence,
                scan_distance_p95_m=p95, scan_endpoints=len(distances), scan_stamp_tf_lookups=lookups, lookup_errors=errors,
                checks=dict(position_and_yaw_converged=convergence is not None and convergence <= tol['convergence_sim_s'] and bool(ok[-3:].all()),
                            scan_map_aligned=p95 is not None and p95 <= tol['scan_distance_p95_max_m'],
                            scan_stamp_tf_available=lookups >= 7 and not errors))


def main():
    meta = yaml.safe_load((WS/'src/t1_navigation/maps/static_world.meta.yaml').read_text())
    path = WS/'src/t1_navigation/maps/static_world.yaml'
    checksum = digest(path)
    assert checksum == meta['map_yaml_sha256']
    session = Session('localization')
    results, run = [], None
    try:
        for index, initial in enumerate(meta['expected_localization']['initial_inputs']):
            run = Run('localization.launch.py', f'localization_{index}',
                      ['scene:=static_world', f'rviz:={str(index == 0).lower()}', 'max_sim_time:=90.0'])
            run.ready()
            run.until(lambda: run.node.map is not None, 60)
            states = {n: run.call(GetState, f'/{n}/get_state', GetState.Request()).current_state.label for n in ('map_server', 'amcl')}
            assert all(v == 'active' for v in states.values()), states
            run.arm_teleop()
            command = ['ros2', 'run', 't1_bringup', 'initial_pose', *[str(v) for v in initial]]
            initial_sim = run.node.sim
            proc = run.start_child(command, 'init')
            run.until(lambda: proc.poll() is not None, 30)
            assert proc.returncode == 0, 'initial_pose failed'
            run.until(lambda: run.node.pose is not None and run.node.map_tf is not None)
            for command_pair, seconds in (((.4, 0.), 4.), ((0., 0.), 2.), ((0., .6), 3.), ((0., 0.), 2.),
                                          ((.4, 0.), 2.), ((0., 0.), 3.)):
                run.source = command_pair
                run.duration(seconds)
            measurements = evaluate(run, initial_sim, meta)
            graph = run.node.graph(('/map', '/tf'))
            checks = measurements.pop('checks')
            rows = run.node.rows
            checks.update(map_server_only=graph['/map']['publishers'] == ['map_server'],
                          no_slam='slam_toolbox' not in graph['/tf']['publishers'],
                          same_map_checksum=digest(path) == checksum,
                          loaded_grid_resolution=abs(run.node.map.info.resolution - .05) < 1e-8,
                          no_clock_jump=run.node.clock_regressions == 0,
                          physical_motion=math.dist((rows[-1]['x'], rows[-1]['y']), (rows[0]['x'], rows[0]['y'])) > .5,
                          gate_stayed_armed=run.node.gate['state'] == 'ARMED')
            if index == 0:
                run.screenshot(OUT/'localization.png')
            closed = run.close()
            run = None
            checks.update(one_initialization=len(closed['initializations']) == 1,
                          actual_amcl_map_odom_owner=closed['tf_owners'].get('odom') == ['amcl'],
                          clean_exit=closed['exit_code'] == 0 and closed['manifest'].get('exit_code') == 0,
                          bound_to_map=closed['manifest'].get('map_sha256') == checksum)
            checks = {k: bool(v) for k, v in checks.items()}
            result = dict(index=index, initial_pose_map=initial, initial_pose_command=command, initial_sim=initial_sim,
                          lifecycle_states=states, map_yaml_sha256=checksum, expected=meta['expected_localization'],
                          measurements=measurements, checks=checks, run=closed,
                          verdict='PASS' if all(checks.values()) else 'FAIL')
            results.append(result)
            (OUT/f'localization-{index}.json').write_text(json.dumps(result, indent=2) + '\n')
            print('LOCALIZATION', index, result['verdict'], measurements, flush=True)
    except Exception as exc:
        results.append(dict(verdict='FAIL', error=repr(exc)))
        print('ERROR', repr(exc), flush=True)
    finally:
        if run is not None:
            results.append(dict(interrupted_run=run.close(), verdict='FAIL'))
        session.close()
        report = dict(verdict='PASS' if len(results) == 3 and all(r['verdict'] == 'PASS' for r in results) else 'FAIL',
                      trials=[dict(index=r.get('index'), verdict=r['verdict'], error=r.get('error'),
                                   measurements=r.get('measurements'), checks=r.get('checks')) for r in results])
        (OUT/'localization.json').write_text(json.dumps(report, indent=2) + '\n')
    print('VERDICT', report['verdict'], flush=True)
    return 0 if report['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

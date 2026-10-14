"""Three independent AMCL startups. One approximate initialization per startup."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time
from run_checks import WS, OUT, RAW, Run, yaw, ns, angle, digest
import yaml


def evaluate(run, initial_sim, meta):
    rows=np.array(run.node.rows)
    poses=np.array(run.node.poses)
    assert len(poses)>5, 'AMCL pose updates missing'
    transform=meta['T_world_map'];a=transform['yaw'];c,s=math.cos(a),math.sin(a)
    R=np.array([[c,-s],[s,c]])
    xy=poses[:,1:3]@R.T+np.array([transform['x'],transform['y']])
    truth=np.c_[np.interp(poses[:,0],rows[:,0],rows[:,1]),np.interp(poses[:,0],rows[:,0],rows[:,2])]
    true_yaw=np.interp(poses[:,0],rows[:,0],np.unwrap(rows[:,3]))
    pos=np.linalg.norm(xy-truth,axis=1)
    rotation=np.abs(np.arctan2(np.sin(poses[:,3]+a-true_yaw),np.cos(poses[:,3]+a-true_yaw)))
    tolerance=meta['expected_localization'];ok=(pos<=tolerance['position_error_max_m']) & (rotation<=tolerance['yaw_error_max_rad'])
    # Require final updates to stay inside tolerance and report first converged update.
    convergence=next((float(poses[i,0]-initial_sim) for i in range(len(poses)) if ok[i:].all()), None)
    np.savetxt(OUT/(run.run_id+'-pose-errors.csv'),np.c_[poses[:,0],pos,rotation],delimiter=',',
               header='sim_time,position_error_m,yaw_error_rad',comments='')
    grid=run.node.map
    data=np.array(grid.data).reshape(grid.info.height,grid.info.width)
    iy,ix=np.where(data>=65)
    occupied=np.c_[ix+.5,iy+.5]*grid.info.resolution+np.array([grid.info.origin.position.x,grid.info.origin.position.y])
    distances=[];lookups=0;lookup_errors=[]
    for scan in run.node.scans[-8:]:
        try:
            tf=run.node.buffer.lookup_transform('map','laser_stabilized',Time(nanoseconds=ns(scan.header.stamp)))
        except Exception as exc:
            lookup_errors.append(str(exc));continue
        t=tf.transform;p=t.translation;theta=yaw(t.rotation)
        ranges=np.array(scan.ranges);valid=np.isfinite(ranges)
        angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment+theta
        points=np.c_[ranges[valid]*np.cos(angles[valid])+p.x,ranges[valid]*np.sin(angles[valid])+p.y]
        distance=np.sqrt(np.min(np.sum((points[:,None,:]-occupied[None,:,:])**2,axis=2),axis=1))
        distances.extend(distance);lookups+=1
    p95=float(np.percentile(distances,95)) if distances else None
    return dict(pose_updates=len(poses),position_error_final_m=float(pos[-1]),yaw_error_final_rad=float(rotation[-1]),
                yaw_error_final_deg=float(math.degrees(rotation[-1])),convergence_sim_s=convergence,
                scan_distance_p95_m=p95,scan_endpoints=len(distances),scan_stamp_tf_lookups=lookups,lookup_errors=lookup_errors,
                checks=dict(position_and_yaw_converged=convergence is not None and convergence<=tolerance['convergence_sim_s'] and bool(ok[-3:].all()),
                            scan_map_aligned=p95 is not None and p95<=tolerance['scan_distance_p95_max_m'],
                            scan_stamp_tf_available=lookups>=7 and not lookup_errors))


def main():
    meta=yaml.safe_load((WS/'src/t1_navigation/maps/static_world.meta.yaml').read_text())
    path=WS/'src/t1_navigation/maps/static_world.yaml'
    checksum=digest(path)
    assert checksum==meta['map_yaml_sha256']
    server_log=(RAW/'localization-dds.txt').open('w')
    server=subprocess.Popen([str(WS/'scripts/start_local_dds.sh')],stdout=server_log,stderr=subprocess.STDOUT)
    rclpy.init()
    results=[];run=None
    try:
        time.sleep(.4);assert server.poll() is None
        for index,initial in enumerate(meta['expected_localization']['initial_inputs']):
            run=Run('localization',f'localization_{index}',rviz=index==0)
            run.ready()
            run.until(lambda: run.node.map is not None and run.node.init_pub.get_subscription_count()>0)
            states={name:run.call(GetState,f'/{name}/get_state',GetState.Request()).current_state.label for name in ('map_server','amcl')}
            assert all(v=='active' for v in states.values()),states
            # Exercise the actual user-facing one-shot initial_pose executable.
            command=['ros2','run','t1_bringup','initial_pose',*[str(v) for v in initial]]
            initial_sim=run.node.sim
            log=(RAW/(run.run_id+'-init.txt')).open('w')
            proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            run.until(lambda: proc.poll() is not None,timeout=25)
            log.close();assert proc.returncode==0,'initial_pose failed'
            run.until(lambda: run.node.pose is not None and run.node.map_tf is not None)
            run.duration(4.,(.4,0.))
            run.duration(2.)
            run.duration(3.,(0.,.6))
            run.duration(2.)
            run.duration(2.,(.4,0.))
            run.duration(3.)
            run.until(lambda: run.node.map_tf is not None)
            measurements=evaluate(run,initial_sim,meta)
            graph=run.node.graph()
            checks=measurements.pop('checks')
            checks.update(map_server_only=all(i['node']=='map_server' for i in graph['/map']) and len(graph['/map'])==1,
                          no_slam=not any(i['node']=='slam_toolbox' for i in graph['/tf']),
                          same_map_checksum=digest(path)==checksum,
                          loaded_grid_resolution=abs(run.node.map.info.resolution-.05)<1e-8,
                          no_clock_jump=run.node.clock_regressions==0,
                          physical_motion=np.linalg.norm(np.array(run.node.rows[-1][1:3])-np.array(run.node.rows[0][1:3]))>.5)
            if index==0:run.screenshot('localization.png')
            closed=run.close();run=None
            checks.update(one_initialization=len(closed['initializations'])==1, actual_amcl_map_odom_owner=len(closed['map_odom_gids'])==1 and closed['map_odom_nodes']==['amcl'],
                          clean_exit=closed['exit_code']==0 and closed['manifest'].get('exit_code')==0)
            result=dict(index=index,initial_pose_map=initial,initial_pose_publish_count=len(closed['initializations']),initial_pose_command=command,
                        initial_sim=initial_sim,lifecycle_states=states,map_yaml_sha256=checksum,expected=meta['expected_localization'],
                        measurements=measurements,checks=checks,run=closed,verdict='PASS' if all(checks.values()) else 'FAIL')
            results.append(result)
            (OUT/f'localization-{index}.json').write_text(json.dumps(result,indent=2)+'\n')
            print('LOCALIZATION',index,result['verdict'],measurements,flush=True)
            time.sleep(.5)
    except Exception as exc:
        results.append(dict(verdict='FAIL',error=repr(exc)))
        print('ERROR',repr(exc),flush=True)
    finally:
        if run:
            results.append(dict(interrupted_run=run.close(),verdict='FAIL'))
        server.send_signal(signal.SIGINT);server.wait(timeout=10);server_log.close();rclpy.shutdown()
        report=dict(verdict='PASS' if len(results)==3 and all(r['verdict']=='PASS' for r in results) else 'FAIL',trials=results)
        (OUT/'localization.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['verdict']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())

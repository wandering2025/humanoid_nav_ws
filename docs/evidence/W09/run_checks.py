"""Actual W09 ROS experiments. Predetermined inspection route, no Nav2 planner.

Run with configured venv after sourcing scripts/env.sh, T1_LOCAL_DDS=1.
Each invocation owns its discovery server and launch process; logs are ignored.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from slam_toolbox.srv import SerializePoseGraph

WS = Path(os.environ['T1_WS'])
OUT = WS / 'docs/evidence/W09'
RAW = OUT / 'raw'
RAW.mkdir(exist_ok=True)


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Probe(Node):
    def __init__(self):
        super().__init__('w09_probe')
        self.sim = 0.
        self.state = self.odom = self.scan = self.map = self.map_tf = self.pose = None
        self.buffer = Buffer()
        self.rows = []
        self.map_updates = []
        self.map_gids = set()
        self.scans = []
        self.poses = []
        self.initializations = []
        self.tf_owners = {}
        self.clock_regressions = 0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(Clock, '/clock', self.clock, 1000)
        self.create_subscription(String, '/t1/sim_state', self.status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/odom', self.odometry, 1000)
        self.create_subscription(LaserScan, '/scan', self.laser, 100)
        self.create_subscription(OccupancyGrid, '/map', self.grid,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(TFMessage, '/tf', self.tf, 1000)
        self.create_subscription(TFMessage, '/tf_static', self.static,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', lambda m: self.initializations.append([m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)]), 100)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def clock(self, m):
        value = ns(m.clock)*1e-9
        self.clock_regressions += value < self.sim
        self.sim = value

    def status(self, m):
        self.state = json.loads(m.data)
        assert self.state['state'] not in ('FAULT', 'FALLEN'), self.state

    def odometry(self, m):
        self.odom = m
        p = m.pose.pose.position
        self.rows.append([ns(m.header.stamp)*1e-9, p.x, p.y, yaw(m.pose.pose.orientation),
                          m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z])

    def laser(self, m):
        self.scan = m
        # Preserve a sparse history for geometric overlay and process evidence.
        if len(self.scans) == 0 or ns(m.header.stamp) - ns(self.scans[-1].header.stamp) >= 500_000_000:
            self.scans.append(m)

    def grid(self, m, info):
        self.map = m
        self.map_gids.add('map topic graph')
        self.map_updates.append([self.sim, ns(m.header.stamp)*1e-9, m.info.width, m.info.height,
                                 sum(v >= 65 for v in m.data)])

    def tf(self, m, info):
        gid = 'observed child; actual RMW authority checked by C++ sidecar'
        for t in m.transforms:
            self.tf_owners.setdefault(t.child_frame_id, set()).add(gid)
            self.buffer.set_transform(t, gid)
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.map_tf = t

    def static(self, m):
        for t in m.transforms:
            self.buffer.set_transform_static(t, 'rsp')

    def amcl(self, m):
        self.pose = m
        p = m.pose.pose.position
        self.poses.append([ns(m.header.stamp)*1e-9, p.x, p.y, yaw(m.pose.pose.orientation)])

    def cmd(self, vx=0., wz=0.):
        m = Twist()
        m.linear.x, m.angular.z = float(vx), float(wz)
        self.pub.publish(m)

    def graph(self):
        return {topic: [dict(node=i.node_name, gid=bytes(i.endpoint_gid).hex(), type=i.topic_type)
                        for i in self.get_publishers_info_by_topic(topic)]
                for topic in ('/tf', '/clock', '/scan', '/odom', '/map', '/cmd_vel', '/initialpose')}


class Run:
    def __init__(self, mode, label, rviz=False):
        self.run_id = f'w09_{label}_{uuid.uuid4().hex[:8]}'
        self.node = Probe()
        self.command = ['ros2', 'launch', 't1_bringup', f'{mode}.launch.py',
                        f'run_id:={self.run_id}', 'scene:=static_world', 'target_rtf:=1.0',
                        f'rviz:={str(rviz).lower()}', 'max_sim_time:=220.0']
        self.log_path = RAW / (self.run_id+'.txt')
        self.log = self.log_path.open('w')
        self.proc = subprocess.Popen(self.command, cwd=WS, stdout=self.log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL)
        self.authority_path = RAW/(self.run_id+'-authority.jsonl')
        self.authority_log = self.authority_path.open('w')
        self.authority = subprocess.Popen([str(RAW/'authority-build/tf_authority')], stdout=self.authority_log, stderr=subprocess.DEVNULL)
        self.wall0 = time.monotonic()
        self.started = datetime.now(timezone.utc).isoformat()
        self.next_pub = 0.
        self.events = []
        print('RUN', self.run_id, 'PID', self.proc.pid, flush=True)

    def spin(self, command=(0., 0.)):
        assert self.proc.poll() is None, f'launch exited; see {self.log_path}'
        assert time.monotonic()-self.wall0 < 270, 'overall wall deadline'
        rclpy.spin_once(self.node, timeout_sec=.005)
        now = time.monotonic()
        if now >= self.next_pub:
            self.node.cmd(*command)
            self.next_pub = now + .02

    def until(self, predicate, command=(0., 0.), timeout=25):
        end = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < end, 'condition deadline'
            self.spin(command)

    def duration(self, seconds, command=(0., 0.)):
        end = self.node.sim + seconds
        self.until(lambda: self.node.sim >= end, command, timeout=seconds*1.5+15)

    def ready(self):
        self.until(lambda: self.node.state and self.node.state['state']=='READY' and self.node.odom and self.node.scan)
        self.duration(1.)

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        self.until(lambda: client.service_is_ready())
        future = client.call_async(request)
        self.until(future.done, timeout=15)
        result = future.result()
        self.node.destroy_client(client)
        return result

    def screenshot(self, name):
        self.duration(.5)
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        windows = [line.split()[0] for line in tree.splitlines() if '"' in line and 'RViz' in line and 'rviz2' in line.lower()]
        assert windows, 'RViz window missing'
        subprocess.run(['import', '-window', windows[-1], str(OUT/name)], check=True, timeout=10)

    def route(self):
        # Manually selected inspection points; transitions use observed odom.
        # No environmental truth, costmap, planner or autonomous navigation goal.
        points = [(0., 1.9), (6.8, 1.9), (6., 0.), (6.2, -2.), (0., -2.), (0., 0.)]
        for i, target in enumerate(points):
            print('INSPECTION', i, target, 'at', self.node.rows[-1][:4], flush=True)
            start = self.node.sim
            p = self.node.odom.pose.pose.position
            heading = math.atan2(target[1]-p.y, target[0]-p.x)
            # Compensate stopping rotation conservatively, then settle.
            def aligned():
                return abs(angle(heading-yaw(self.node.odom.pose.pose.orientation))) < .16
            self.until(aligned, (0., .6 * (1 if angle(heading-yaw(self.node.odom.pose.pose.orientation)) > 0 else -1)), timeout=18)
            self.duration(2.)
            deadline = time.monotonic()+40
            while True:
                p = self.node.odom.pose.pose.position
                distance = math.hypot(target[0]-p.x, target[1]-p.y)
                if distance < .23:
                    break
                assert time.monotonic() < deadline, 'inspection leg deadline'
                desired = math.atan2(target[1]-p.y, target[0]-p.x)
                error = angle(desired-yaw(self.node.odom.pose.pose.orientation))
                self.spin((.4, max(-.4, min(.4, error*1.6))))
            self.duration(2.)
            self.events.append(dict(leg=i, target=target, sim_start=start, sim_end=self.node.sim,
                                    actual=self.node.rows[-1][:4]))
            if i in (1, 3):
                self.screenshot(f'mapping-leg-{i}.png')
            print('ARRIVED', self.node.rows[-1][:4], flush=True)

    def close(self):
        graph = self.node.graph()
        if self.proc.poll() is None:
            # Send zero, wait for physical settling before closing own launch.
            try:
                self.duration(2.)
            except Exception:
                pass
            self.proc.send_signal(signal.SIGINT)
        code = self.proc.wait(timeout=15)
        self.log.close()
        manifest_path = WS/'run_data'/self.run_id/'manifest.json'
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        owners = self.node.tf_owners.get('odom', set())
        identities = {x['gid']: x['node'] for x in graph['/tf']}
        result = dict(run_id=self.run_id, command=self.command, cwd=str(WS), started_utc=self.started,
                      ended_utc=datetime.now(timezone.utc).isoformat(), exit_code=code,
                      wall_s=time.monotonic()-self.wall0, sim_end=self.node.sim, manifest=manifest,
                      initializations=self.node.initializations, graph=graph, map_odom_gids=list(owners), map_odom_nodes=[identities.get(g,'exited') for g in owners],
                      map_updates=self.node.map_updates, route=self.events, clock_regressions=self.node.clock_regressions)
        np.savetxt(OUT/(self.run_id+'-trajectory.csv'), np.array(self.node.rows)[::10], delimiter=',',
                   header='sim_time,x,y,yaw,vx,vy,wz', comments='')
        (RAW/(self.run_id+'-poses.json')).write_text(json.dumps(self.node.poses))
        self.authority.send_signal(signal.SIGINT)
        self.authority.wait(timeout=10)
        self.authority_log.close()
        records=[json.loads(line) for line in self.authority_path.read_text().splitlines()]
        result['tf_authority']=records
        actual=[r for r in records if r['parent']=='map' and r['child']=='odom']
        result['map_odom_gids']=sorted({r['gid'] for r in actual})
        result['map_odom_nodes']=sorted({r['node'] for r in actual if r['node']!='unresolved'})
        self.node.destroy_node()
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('phase', choices=['c1', 'mapping'])
    opts = parser.parse_args()
    assert os.environ.get('T1_LOCAL_DDS') == '1'
    server_log = (RAW/f'{opts.phase}-dds.txt').open('w')
    server = subprocess.Popen([str(WS/'scripts/start_local_dds.sh')], stdout=server_log, stderr=subprocess.STDOUT)
    rclpy.init()
    run = None
    result = {}
    try:
        time.sleep(.4)
        assert server.poll() is None, 'discovery server failed'
        if opts.phase in ('c1', 'mapping'):
            run = Run('mapping', opts.phase, rviz=opts.phase=='mapping')
            run.ready()
            run.until(lambda: run.node.map is not None and run.node.map_tf is not None)
            graph = run.node.graph()
            reset = run.call(Trigger, '/t1/sim/reset', Trigger.Request())
            checks = dict(map_received=run.node.map is not None,
                          slam_only=any(i['node']=='slam_toolbox' for i in graph['/tf']) and not any(i['node']=='amcl' for i in graph['/tf']),
                          one_map_odom_owner=len(run.node.tf_owners.get('odom',set()))==1,
                          one_clock_owner=len(graph['/clock'])==1,
                          reset_rejected=not reset.success,
                          sensor_frame=run.node.scan.header.frame_id=='laser_stabilized')
            t=run.node.map_tf.transform
            result.update(checks=checks, graph_active=graph, initial_map_odom=[t.translation.x,t.translation.y,yaw(t.rotation)],
                          initial_odom=run.node.rows[-1][:4], initial_map_tf_stamp=ns(run.node.map_tf.header.stamp)*1e-9)
            if opts.phase == 'mapping':
                run.route()
                run.duration(3.)
                run.screenshot('mapping-final.png')
                m=run.node.map
                np.savez_compressed(RAW/'slam-grid.npz', grid=np.array(m.data).reshape(m.info.height,m.info.width), resolution=m.info.resolution, origin=[m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation)])
                t=run.node.map_tf.transform
                result['final_map_odom']=[t.translation.x,t.translation.y,yaw(t.rotation)]
                mapbase = WS/'src/t1_navigation/maps/static_world'
                cmd = ['ros2','run','nav2_map_server','map_saver_cli','-f',str(mapbase),'--ros-args','-p','use_sim_time:=true','-p','save_map_timeout:=10.0']
                saved = subprocess.Popen(cmd, stdout=(RAW/'map-save.txt').open('w'), stderr=subprocess.STDOUT)
                run.until(lambda: saved.poll() is not None, timeout=20)
                assert saved.returncode==0, 'map save failed'
                request = SerializePoseGraph.Request(filename=str(RAW/'static_world'))
                response = run.call(SerializePoseGraph,'/slam_toolbox/serialize_map', request)
                assert response.result == 0, 'graph serialization failed'
                result['map_save_command'] = cmd
                result['serialized_pose_graph'] = str(RAW/'static_world')
        result['verdict'] = 'PASS' if all(result['checks'].values()) else 'FAIL'
    except Exception as exc:
        result.update(verdict='FAIL', error=repr(exc))
        print(result['error'], flush=True)
    finally:
        if run:
            result['run'] = run.close()
            expected='slam_toolbox'
            result.setdefault('checks', {})['actual_map_odom_owner']=(len(result['run']['map_odom_gids'])==1 and result['run']['map_odom_nodes']==[expected])
            result['checks']['clean_exit']=result['run']['exit_code']==0 and result['run']['manifest'].get('exit_code')==0
            if not all(result['checks'].values()):
                result['verdict']='FAIL'
        server.send_signal(signal.SIGINT)
        server.wait(timeout=10)
        server_log.close()
        rclpy.shutdown()
        (OUT/(opts.phase+'.json')).write_text(json.dumps(result, indent=2)+'\n')
    return 0 if result.get('verdict')=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

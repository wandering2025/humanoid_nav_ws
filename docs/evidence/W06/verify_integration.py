"""W06 real launch/ROS acceptance. Own processes only; no MjData access.

Run with sourced scripts/env.sh, T1_LOCAL_DDS=1 and unused T1_DDS_PORT.
Each invocation starts/stops its dedicated discovery server. --suite controls
includes pause/resume/reset plus a full restart with new consumer/TF buffer.
--suite motion physically turns then walks; --rviz captures actual RViz windows.
"""
import argparse
from collections import defaultdict
import csv
from datetime import datetime, timezone
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from t1_bringup.description import robot_description
import xml.etree.ElementTree as ET

WS = Path(os.environ['T1_WS'])
OUT = WS / 'docs/evidence/W06'
RAW = OUT / 'raw'


def ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def xyz(v):
    return [v.x, v.y, v.z]


def tf_source(message, static=False):
    # Jazzy MessageInfo has timestamps/sequence numbers, no publisher GID.
    # Cross-check complete broadcaster batches against disjoint ownership sets.
    children = {t.child_frame_id for t in message.transforms}
    if not static and children == {'base_footprint', 'base_link'}:
        return 't1_sim (batch + graph ownership)'
    assert not children & {'base_footprint', 'base_link'}, children
    return 'robot_state_publisher (URDF batch + graph ownership)'


class Probe(Node):
    def __init__(self, run_id):
        super().__init__('w06_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.run_id = run_id
        self.sim = 0.
        self.state = None
        self.odom = None
        self.rows = defaultdict(list)
        self.parents, self.authorities = {}, defaultdict(set)
        self.static = {}
        self.buffer = Buffer(node=self)
        self.epoch = None
        self.epoch_changed_at = 0.
        self.lookup_results = []
        self.topic_info = {}
        self.create_subscription(Clock, '/clock', self.clock, 1000)
        self.create_subscription(Odometry, '/odom', self.on_odom, 1000)
        self.create_subscription(Odometry, '/t1/eval/ground_truth', self.truth, 1000)
        self.create_subscription(JointState, '/joint_states', self.joints, 1000)
        self.create_subscription(TFMessage, '/tf', self.tf, 1000)
        self.create_subscription(TFMessage, '/tf_static', self.tf_static,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/t1/sim_state', self.status,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pause_client = self.create_client(SetBool, '/t1/sim/pause')
        self.reset_client = self.create_client(Trigger, '/t1/sim/reset')

    def clock(self, m):
        self.sim = ns(m.clock)*1e-9
        self.rows['/clock'].append([ns(m.clock), time.monotonic()])

    def status(self, m):
        s = json.loads(m.data)
        if s.get('run_id') != self.run_id:
            return
        self.state = s
        if self.epoch != s['episode']:
            self.epoch = s['episode']
            self.epoch_changed_at = self.sim
            self.buffer.clear()
            for tf, source in self.static.values():
                self.buffer.set_transform_static(tf, source)
        self.rows['state'].append(s)

    def on_odom(self, m):
        self.odom = m
        self.rows['/odom'].append([ns(m.header.stamp), *xyz(m.pose.pose.position), yaw(m.pose.pose.orientation),
                                   *xyz(m.twist.twist.linear), *xyz(m.twist.twist.angular)])
        assert m.header.frame_id == 'odom' and m.child_frame_id == 'base_footprint'

    def truth(self, m):
        self.rows['/t1/eval/ground_truth'].append([ns(m.header.stamp), *xyz(m.pose.pose.position),
                                                 m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                                                 m.pose.pose.orientation.y, m.pose.pose.orientation.z])
        assert m.header.frame_id == 'mj_world' and m.child_frame_id == 'base_link_truth'

    def joints(self, m):
        self.rows['/joint_states'].append([ns(m.header.stamp), list(m.name), list(m.position), list(m.velocity), list(m.effort)])

    def remember(self, tf, source):
        child = tf.child_frame_id
        parent = tf.header.frame_id
        assert child not in self.parents or self.parents[child] == parent, (child, parent)
        self.parents[child] = parent
        self.authorities[child].add(source)
        assert child not in ('map', 'odom', 'mj_world', 'base_link_truth')

    def tf(self, m, info):
        source = tf_source(m)
        for tf in m.transforms:
            self.remember(tf, source)
            self.buffer.set_transform(tf, source)
            self.rows['/tf:'+tf.child_frame_id].append(ns(tf.header.stamp))
            if tf.child_frame_id == 'base_link':
                self.rows['base_tf'].append([ns(tf.header.stamp), *xyz(tf.transform.translation),
                    tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z])

    def tf_static(self, m, info):
        source = tf_source(m, static=True)
        for tf in m.transforms:
            self.remember(tf, source)
            self.static[tf.child_frame_id] = (tf, source)
            self.buffer.set_transform_static(tf, source)

    def command(self, vx=0., wz=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(vx), float(wz)
        self.pub.publish(msg)

    def lookup(self):
        if self.odom is None or self.sim-self.epoch_changed_at < .4:
            return
        t = Time(nanoseconds=max(0, ns(self.odom.header.stamp)-100_000_000))
        try:
            for target in ['base_footprint', 'base_link', 'trunk', 'left_ankle_roll_link']:
                self.buffer.lookup_transform('odom', target, t)
            self.lookup_results.append([self.sim, True, ''])
        except TransformException as exc:
            self.lookup_results.append([self.sim, False, str(exc)])

    def graph(self):
        for topic in ['/clock','/odom','/joint_states','/t1/eval/ground_truth','/tf','/tf_static','/robot_description','/imu/data']:
            self.topic_info[topic] = dict(publishers=[dict(node=i.node_name, type=i.topic_type, gid=bytes(i.endpoint_gid).hex(),
                    reliability=i.qos_profile.reliability.name, durability=i.qos_profile.durability.name,
                    depth=i.qos_profile.depth) for i in self.get_publishers_info_by_topic(topic)],
                    subscribers=[i.node_name for i in self.get_subscriptions_info_by_topic(topic)])


class Run:
    def __init__(self, label, rviz=False):
        self.run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + f'_dev_w06_{label}_s0_' + uuid.uuid4().hex[:6]
        self.command = ['ros2','launch','t1_bringup','simulation.launch.py',f'run_id:={self.run_id}',
                        'target_rtf:=1.0', 'max_sim_time:=55.0',f'rviz:={str(rviz).lower()}']
        self.started = datetime.now(timezone.utc).isoformat()
        self.wall0 = time.monotonic()
        self.log_path = RAW / (self.run_id+'.txt')
        self.log = self.log_path.open('w')
        self.node = Probe(self.run_id)
        self.proc = subprocess.Popen(self.command,cwd=WS,stdin=subprocess.DEVNULL,stdout=self.log,stderr=subprocess.STDOUT)
        self.next_lookup = self.next_pub = 0.
        print('RUN',self.run_id,'launch_pid',self.proc.pid,flush=True)

    def spin(self, command=None):
        assert self.proc.poll() is None, f'launch exited {self.proc.returncode}; {self.log_path}'
        assert time.monotonic()-self.wall0 < 75, 'run deadline exceeded'
        rclpy.spin_once(self.node, timeout_sec=.005)
        now=time.monotonic()
        if now>=self.next_pub:
            if command is not None:
                self.node.command(*command)
            self.next_pub=now+.02
        if now>=self.next_lookup:
            self.node.lookup()
            self.next_lookup=now+.1

    def until(self, predicate, command=None, timeout=20):
        deadline=time.monotonic()+timeout
        while not predicate():
            assert time.monotonic()<deadline, 'condition deadline exceeded'
            self.spin(command)

    def duration(self, seconds, command=None, wall=False):
        end=(time.monotonic() if wall else self.node.sim)+seconds
        self.until(lambda: (time.monotonic() if wall else self.node.sim)>=end,command,timeout=seconds+10)

    def ready(self):
        self.until(lambda: self.node.state is not None and self.node.state['state']=='READY' and self.node.odom is not None)
        self.duration(1.,(0.,0.))
        self.node.graph()

    def service(self, client, request):
        assert client.wait_for_service(timeout_sec=3)
        future=client.call_async(request)
        self.until(future.done, timeout=4)
        result=future.result()
        assert result.success,result.message
        self.duration(.2,wall=True)
        return result.message

    def screenshot(self, name):
        tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
        windows=[line.split()[0] for line in tree.splitlines() if '"' in line and 'RViz' in line and 'rviz2' in line.lower()]
        assert windows,tree
        path=OUT/name
        subprocess.run(['import','-window',windows[-1],str(path)],check=True,timeout=10)
        return name

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        code=self.proc.wait(timeout=12)
        self.log.close()
        # Drain final messages before the consumer is destroyed.
        for _ in range(30):
            rclpy.spin_once(self.node, timeout_sec=.002)
        result=dict(run_id=self.run_id,command=self.command,cwd=str(WS),started_utc=self.started,
                    ended_utc=datetime.now(timezone.utc).isoformat(),wall_s=time.monotonic()-self.wall0,
                    exit_code=code,log=str(self.log_path.relative_to(OUT)))
        manifest_path=WS/'run_data'/self.run_id/'manifest.json'
        manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        result['manifest']=manifest
        try:
            result.update(analyse(self.node))
        except Exception as exc:
            result.update(analysis_error=repr(exc),checks={'analysis_completed':False})
        (RAW/(self.run_id+'-received.json')).write_text(json.dumps(dict(self.node.rows))+'\n')
        (RAW/(self.run_id+'-meta.json')).write_text(json.dumps(dict(topic_info=self.node.topic_info, parents=self.node.parents, authorities={k:list(v) for k,v in self.node.authorities.items()},lookup_results=self.node.lookup_results))+'\n')
        self.node.destroy_node()
        (OUT/(self.run_id+'-result.json')).write_text(json.dumps(result,indent=2)+'\n')
        return result


def analyse(node):
    rows=node.rows
    topics={}
    for topic in ['/clock','/odom','/joint_states','/t1/eval/ground_truth','/tf:base_footprint','/tf:base_link','/tf:left_ankle_roll_link']:
        stamps=[int(r if isinstance(r,int) else r[0]) for r in rows[topic]]
        unique=sorted(set(stamps))
        deltas=np.diff(unique)*1e-9
        topics[topic]=dict(count=len(stamps),stamp_min_ns=min(stamps) if stamps else None,
                          stamp_max_ns=max(stamps) if stamps else None,
                          median_sim_hz=float(1/np.median(deltas)) if len(deltas) else None,
                          max_gap_s=float(max(deltas)) if len(deltas) else None,
                          regressions=sum(b<a for a,b in zip(stamps,stamps[1:])))
    odom={r[0]:r for r in rows['/odom']}
    truth={r[0]:r for r in rows['/t1/eval/ground_truth']}
    joints={r[0]:r for r in rows['/joint_states']}
    tf=set(rows['/tf:base_footprint'])
    common=sorted(set(odom)&set(truth)&set(joints)&tf&set(rows['/tf:base_link']))
    pos_error=max(math.hypot(odom[t][1]-truth[t][1],odom[t][2]-truth[t][2]) for t in common)
    expected_root=ET.fromstring(robot_description(os.environ.get('T1_ASSETS',str(Path(os.environ['T1_UPSTREAM']).parent/'booster_assets'))))
    expected={j.attrib['name'] for j in expected_root.findall('joint') if j.attrib['type']!='fixed'}
    joint_ok=all(set(r[1])==expected and len(r[1])==23 and all(len(a)==23 for a in r[2:]) and np.isfinite(np.array(r[2:])).all() for r in joints.values())
    # World velocity recovered from each odom child twist, trapezoid integration.
    ordered=[odom[t] for t in sorted(odom)]
    integral=np.zeros(2);delta=np.zeros(2);max_error=0.;pairs=0
    for a,b in zip(ordered,ordered[1:]):
        dt=(b[0]-a[0])*1e-9
        # Exclude sim_only reset jump (pose discontinuity), never integrate across it.
        if dt>.021 or math.hypot(b[1]-a[1],b[2]-a[2])>.1:
            integral[:]=0;delta[:]=0
            continue
        def world(r):
            c,s=math.cos(r[4]),math.sin(r[4])
            return np.array([c*r[5]-s*r[6],s*r[5]+c*r[6]])
        integral+=(world(a)+world(b))*.5*dt
        delta+=np.array(b[1:3])-a[1:3]
        max_error=max(max_error,float(np.linalg.norm(integral-delta)))
        pairs+=1
    cycle=False
    for child in node.parents:
        seen=set();current=child
        while current in node.parents:
            if current in seen:
                cycle=True;break
            seen.add(current);current=node.parents[current]
    failures=[r for r in node.lookup_results if not r[1]]
    persistent=[r for i,r in enumerate(node.lookup_results) if i>=2 and not r[1] and not node.lookup_results[i-1][1] and not node.lookup_results[i-2][1]]
    result=dict(topics=topics,graph=node.topic_info,tf_parents=node.parents,
                tf_authorities={k:sorted(v) for k,v in node.authorities.items()},
                authority_method='Jazzy MessageInfo lacks GID: disjoint TF batches + URDF ownership + endpoint graph',
                common_stamps=len(common),max_odom_truth_xy_error_m=pos_error,
                max_integral_error_m=max_error,integrated_pairs=pairs,
                tf_lookup_attempts=len(node.lookup_results),tf_lookup_failures=len(failures),
                tf_lookup_failure_examples=failures[:5],tf_persistent_failures=len(persistent),
                joint_names=sorted(expected),joint_states_valid=bool(joint_ok))
    result['checks']=dict(clock_monotonic=topics['/clock']['regressions']==0,
                         shared_stamps=len(common)>100,
                         odom_truth_match=pos_error<1e-12,integral_within_2cm=max_error<=.02,
                         joint_states_valid=bool(joint_ok),tf_acyclic=not cycle,
                         single_child_authority=all(len(v)==1 for v in node.authorities.values()),
                         no_map_truth_tf=not any(x in node.parents for x in ['map','odom','mj_world','base_link_truth']),
                         tf_queries_recover=len(node.lookup_results)>20 and not persistent,
                         rates_50hz=all(abs(topics[t]['median_sim_hz']-50)<.1 for t in topics),
                         publisher_ownership=all(len(node.topic_info[t]['publishers'])==1 and node.topic_info[t]['publishers'][0]['node']=='t1_sim' for t in ['/clock','/odom','/joint_states','/t1/eval/ground_truth']),
                         two_tf_publishers={p['node'] for p in node.topic_info['/tf']['publishers']}=={'t1_sim','robot_state_publisher'},
                         imu_absent=len(node.topic_info['/imu/data']['publishers'])==0)
    return result


def controls():
    run=Run('controls')
    extra={}
    try:
        run.ready()
        run.duration(2.,(.4,0.))
        run.service(run.node.pause_client,SetBool.Request(data=True))
        run.duration(.3,wall=True)  # drain messages already in flight
        t=run.node.sim;n=len(run.node.rows['/odom'])
        run.duration(1.1,(.4,0.),wall=True)  # commands deliberately sent while paused
        extra['paused_clock_and_odom_frozen']=run.node.sim==t and len(run.node.rows['/odom'])==n
        extra['paused_zero_policy']=run.node.state['u_pol']==[0.,0.,0.]
        run.service(run.node.pause_client,SetBool.Request(data=False))
        run.duration(.8)  # no new commands
        extra['resume_requires_new_command']=run.node.state['u_pol']==[0.,0.,0.] and run.node.state['cmd_status']!='ACTIVE'
        run.duration(1.,(.4,0.))
        extra['fresh_command_works']=run.node.state['u_pol'][0]>0
        run.duration(4.,(0.,0.))
        before=run.node.sim
        old_epoch=run.node.epoch
        run.service(run.node.reset_client,Trigger.Request())
        run.until(lambda: run.node.epoch==old_epoch+1 and run.node.state['state']=='READY')
        extra['reset_episode_notified']=run.node.epoch==old_epoch+1
        extra['reset_time_monotonic']=run.node.sim>before
        extra['reset_command_cleared']=run.node.state['u_pol']==[0.,0.,0.]
        run.duration(2.,(0.,0.))
    finally:
        result=run.close()
    result['checks'].update(extra)
    result['checks']['launch_exit_0']=result['exit_code']==0 and result['manifest']['exit_code']==0
    # Complete graph restart: fresh consumer/buffer, clock starts near zero.
    second=Run('restart')
    try:
        second.ready();second.duration(3.,(0.,0.))
    finally:
        restarted=second.close()
    restarted['checks']['launch_exit_0']=restarted['exit_code']==0 and restarted['manifest']['exit_code']==0
    restarted['checks']['new_clock_origin']=restarted['topics']['/clock']['stamp_min_ns']<1_000_000_000
    return [result,restarted]


def motion(rviz):
    run=Run('motion',rviz)
    detail={}
    try:
        run.ready()
        run.duration(1.,(0.,0.))
        if rviz:
            detail['screenshots']=[run.screenshot('rviz-standing.png')]
        start_yaw=yaw(run.node.odom.pose.pose.orientation)
        run.until(lambda: math.remainder(yaw(run.node.odom.pose.pose.orientation)-start_yaw,math.tau)>=math.radians(112), (0.,.6),timeout=12)
        run.duration(4.,(0.,0.))
        heading=yaw(run.node.odom.pose.pose.orientation)
        begin=list(xyz(run.node.odom.pose.pose.position))
        t_begin=run.node.sim
        run.duration(8.,(.4,0.))
        end=list(xyz(run.node.odom.pose.pose.position))
        t_end=run.node.sim
        if rviz:
            detail['screenshots'].append(run.screenshot('rviz-moving.png'))
        run.duration(4.,(0.,0.))
        samples=[r for r in run.node.rows['/odom'] if t_begin+2<=r[0]*1e-9<=t_end]
        vx=float(np.mean([r[5] for r in samples]))
        dx,dy=end[0]-begin[0],end[1]-begin[1]
        detail.update(heading_after_physical_turn_deg=math.degrees(heading),forward_duration_s=t_end-t_begin,
                      forward_world_displacement_m=[dx,dy],mean_child_vx_m_s=vx,
                      mean_child_vy_m_s=float(np.mean([r[6] for r in samples])))
    finally:
        result=run.close()
    result.update(detail)
    result['checks'].update(launch_exit_0=result['exit_code']==0 and result['manifest']['exit_code']==0,
                           physical_turn_near_90=abs(math.remainder(heading-math.pi/2,math.tau))<math.radians(20),
                           world_direction_rotates=dy>1.5 and abs(dx)<.7,
                           child_vx_forward=vx>.25)
    if rviz:
        result['checks']['rviz_captured']=len(detail['screenshots'])==2
    return [result]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--suite',choices=['controls','motion'],required=True)
    parser.add_argument('--rviz',action='store_true')
    args=parser.parse_args()
    assert os.environ.get('T1_LOCAL_DDS')=='1'
    RAW.mkdir(parents=True,exist_ok=True)
    log=(RAW/(args.suite+'-dds.txt')).open('w')
    server=subprocess.Popen([str(WS/'scripts/start_local_dds.sh')],stdout=log,stderr=subprocess.STDOUT)
    runs=[]
    started=datetime.now(timezone.utc).isoformat()
    rclpy.init()
    try:
        time.sleep(.4)
        assert server.poll() is None,'dedicated server failed'
        runs=controls() if args.suite=='controls' else motion(args.rviz)
    finally:
        rclpy.shutdown()
        if server.poll() is None:
            server.send_signal(signal.SIGINT)
        server.wait(timeout=5);log.close()
        result=dict(started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),
                    environment={k:os.environ.get(k) for k in ['T1_WS','T1_VENV','T1_UPSTREAM','T1_LOCAL_DDS','T1_DDS_PORT','ROS_DOMAIN_ID','ROS_DISCOVERY_SERVER','ROS_SUPER_CLIENT','ROS_LOG_DIR']},
                    runs=runs,server_exit=server.returncode,
                    verdict='PASS' if runs and all(all(r['checks'].values()) for r in runs) else 'FAIL')
        (OUT/(args.suite+'-integration.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps([dict(run_id=r['run_id'],checks=r['checks'],max_integral_error_m=r['max_integral_error_m']) for r in runs],indent=2))
    print('VERDICT',result['verdict'],flush=True)
    return 0 if result['verdict']=='PASS' else 1


if __name__=='__main__':
    raise SystemExit(main())

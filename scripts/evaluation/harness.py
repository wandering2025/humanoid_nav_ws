"""Shared W13 experiment harness: owns one discovery server, one launch, one probe.

Run with the configured venv after `source scripts/env.sh` and T1_LOCAL_DDS=1.
The probe is the evaluator/recorder: it may read /t1/eval/ground_truth; no Nav2,
costmap or localization node does. It never writes MjData or base poses.
"""
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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.msg import CollisionMonitorState
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

WS = Path(os.environ['T1_WS'])
OUT = Path(os.environ['T1_EVAL_OUT'])
RAW = OUT/'raw'
RAW.mkdir(parents=True, exist_ok=True)
AUTHORITY = WS/'docs/evidence/W09/raw/authority-build/tf_authority'
ACTIVE_RUNS = []
LAYERS = (('nav_raw', '/t1/nav/cmd_vel_raw'), ('adapter', '/t1/cmd_navigation'), ('teleop', '/t1/cmd_teleop'),
          ('gate', '/cmd_vel_nav'), ('smoothed', '/cmd_vel_smoothed'), ('final', '/cmd_vel'))


def ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(WS), *args], text=True).strip()


class Probe(Node):
    def __init__(self, name='t1_evaluator', *, dynamic_sample_wait_s=0., tf_queue_depth=1000):
        super().__init__(name)
        self.sim = 0.
        self.clock_regressions = 0
        self.first_clock = None
        self.state = self.gate = self.cm = self.odom = self.truth = self.scan = self.map = None
        self.map_tf = self.pose = self.plan = self.adapter = None
        self.obstacle = None
        self.obstacles, self.dynamic_scans, self.cost_samples = [], [], []
        self.dynamic_pending = None
        if dynamic_sample_wait_s:
            from evaluation.scan_evidence import DeferredSamples
            self.dynamic_pending = DeferredSamples(dynamic_sample_wait_s)
            self.create_timer(.05, self.drain_dynamic_scans)
        self.cmds = {key: None for key, _ in LAYERS}
        self.buffer = Buffer()
        self.rows, self.events, self.scans, self.poses, self.initializations = [], [], [], [], []
        self.map_updates, self.plans = [], []
        self.recoveries = {}
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage,
            '/navigate_to_pose/_action/feedback', self.on_feedback, 10)
        self.scan_stats = dict(count=0, empty=0, known_min=1.0)
        self.last_row = -1.
        self.teleop = None  # created only by runs that act as the teleop source
        qos_tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/t1/eval/obstacle', self.on_obstacle, 50)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_local_costmap, qos_tl)
        self.create_subscription(Clock, '/clock', self.on_clock, 100)
        self.create_subscription(String, '/t1/sim_state', self.on_state, qos_tl)
        self.create_subscription(String, '/t1/safety/state', self.on_gate, qos_tl)
        self.create_subscription(String, '/t1/nav/adapter_state', lambda m: setattr(self, 'adapter', json.loads(m.data)), 10)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.on_cm, 10)
        for key, topic in LAYERS:
            self.create_subscription(Twist, topic, lambda m, k=key: self.cmds.update({k: (m.linear.x, m.angular.z)}), 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 100)
        self.create_subscription(Odometry, '/t1/eval/ground_truth', self.on_truth, 100)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 20)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos_tl)
        # Keep transport backlog bounded separately from tf2's timestamp history.
        # Exact-stamp queries still use the real transforms stored in the buffer.
        self.create_subscription(TFMessage, '/tf', self.on_tf, tf_queue_depth)
        self.create_subscription(TFMessage, '/tf_static', self.on_static,
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 lambda m: self.initializations.append([m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)]), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, qos_tl)
        self.create_subscription(NavPath, '/plan', self.on_plan, 10)

    def on_feedback(self, msg):
        key = bytes(msg.goal_id.uuid).hex()
        self.recoveries[key] = max(self.recoveries.get(key, 0), msg.feedback.number_of_recoveries)

    def event(self, kind, **data):
        self.events.append(dict(kind=kind, sim=self.sim, wall=time.monotonic(), **data))

    def on_clock(self, m):
        value = ns(m.clock)*1e-9
        if self.first_clock is None:
            self.first_clock = value
        self.clock_regressions += value < self.sim
        self.sim = value

    def on_state(self, m):
        data = json.loads(m.data)
        if self.state is None or data['state'] != self.state['state']:
            self.event('sim_state', state=data['state'], fault=data.get('fault'))
        self.state = data

    def on_gate(self, m):
        data = json.loads(m.data)
        if self.gate is None or (data['state'], data['reason']) != (self.gate['state'], self.gate['reason']):
            self.event('gate', state=data['state'], reason=data['reason'])
        self.gate = data

    def on_cm(self, m):
        data = (m.action_type, m.polygon_name)
        if data != self.cm:
            self.event('collision_monitor', action_type=m.action_type, polygon=m.polygon_name)
        self.cm = data

    def on_truth(self, m):
        self.truth = m

    def map_pose(self):
        try:
            t = self.buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))

    def on_odom(self, m):
        self.odom = m
        if self.sim - self.last_row < .0999:
            return
        self.last_row = self.sim
        p, v = m.pose.pose.position, m.twist.twist
        truth = self.truth.pose.pose if self.truth else None
        mp = self.map_pose()
        row = dict(sim=ns(m.header.stamp)*1e-9, wall=time.monotonic(), x=p.x, y=p.y, yaw=yaw(m.pose.pose.orientation),
                   vx=v.linear.x, wz=v.angular.z,
                   tx=truth.position.x if truth else None, ty=truth.position.y if truth else None,
                   tz=truth.position.z if truth else None, tyaw=yaw(truth.orientation) if truth else None,
                   mx=mp[0] if mp else None, my=mp[1] if mp else None, myaw=mp[2] if mp else None,
                   gate=self.gate['state'] if self.gate else None,
                   cm=self.cm[0] if self.cm else None,
                   u_pol=self.state.get('u_pol') if self.state else None,
                   v_guard=self.state.get('v_guard') if self.state else None,
                   cmd_status=self.state.get('cmd_status') if self.state else None,
                   adapter_reason=self.adapter.get('reason') if self.adapter else None)
        for key, _ in LAYERS:
            row[key] = self.cmds[key]
        self.rows.append(row)

    def on_scan(self, m):
        known = sum(not math.isnan(r) for r in m.ranges) / max(1, len(m.ranges))
        finite = [r for r in m.ranges if math.isfinite(r)]
        self.scan_stats['count'] += 1
        self.scan_stats['empty'] += not finite
        self.scan_stats['known_min'] = min(self.scan_stats['known_min'], known)
        self.scan_stats['min_range'] = min(self.scan_stats.get('min_range', math.inf), min(finite) if finite else math.inf)
        self.scan = m
        if self.dynamic_pending is None:
            self.sample_dynamic_scan(m, self.cm)
        else:
            self.dynamic_pending.add((m,self.cm),time.monotonic())
        if not self.scans or ns(m.header.stamp) - ns(self.scans[-1].header.stamp) >= 500_000_000:
            self.scans.append(m)

    def on_map(self, m):
        self.map = m
        self.map_updates.append([self.sim, ns(m.header.stamp)*1e-9, m.info.width, m.info.height])

    def on_tf(self, m):
        for t in m.transforms:
            self.buffer.set_transform(t, 'probe')
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.map_tf = t

    def on_static(self, m):
        for t in m.transforms:
            self.buffer.set_transform_static(t, 'probe')

    def on_amcl(self, m):
        self.pose = m
        p = m.pose.pose.position
        self.poses.append([ns(m.header.stamp)*1e-9, p.x, p.y, yaw(m.pose.pose.orientation)])

    def on_plan(self, m):
        self.plan = m
        if m.poses:
            pts = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
            length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
            self.plans.append(dict(sim=self.sim, n=len(pts), length=length, points=pts[::10] + [pts[-1]]))

    def on_obstacle(self, msg):
        data = json.loads(msg.data)
        self.obstacle = data
        if not self.obstacles or data['sim_time'] - self.obstacles[-1]['sim_time'] >= .0999:
            self.obstacles.append(data)

    def match_obstacle(self, t):
        if not self.obstacles:
            return None
        data = min(self.obstacles[-20:], key=lambda d: abs(d['sim_time']-t))
        return data if abs(data['sim_time']-t) < .2 else None

    def drain_dynamic_scans(self):
        self.dynamic_pending.drain(lambda item:self.sample_dynamic_scan(*item),time.monotonic())

    def sample_dynamic_scan(self, scan, cm_snapshot=None):
        t = ns(scan.header.stamp)*1e-9
        obstacle = self.match_obstacle(t)
        if obstacle is None:
            if self.dynamic_pending is not None:
                self.dynamic_pending.stats['missing_obstacle_attempts'] += 1
            return False
        try:
            tf = self.buffer.lookup_transform('odom', 'laser_stabilized', Time(nanoseconds=ns(scan.header.stamp)))
        except Exception as exc:
            if self.dynamic_pending is not None:
                self.dynamic_pending.stats['missing_tf_attempts'] += 1
                errors = self.dynamic_pending.stats.setdefault('tf_errors', [])
                if len(errors) < 180 and (not errors or t-errors[-1]['scan_sim'] >= 1.):
                    error = dict(scan_sim=t, error=str(exc))
                    errors.append(error)
                    print('dynamic scan TF unavailable: '+json.dumps(error), flush=True)
            return False
        p, a = tf.transform.translation, yaw(tf.transform.rotation)
        hits = []
        x, y, _ = obstacle['position']
        sx, sy, _ = obstacle['size']
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r):
                angle_ = scan.angle_min + i*scan.angle_increment + a
                px, py = p.x + r*math.cos(angle_), p.y + r*math.sin(angle_)
                if abs(px-x) <= sx/2+.06 and abs(py-y) <= sy/2+.06:
                    hits.append([i, r, px, py])
        self.dynamic_scans.append(dict(sim=t, obstacle=obstacle, hits=hits,
                                       laser_pose_odom=[p.x, p.y, p.z, a],
                                       scan_frame=scan.header.frame_id, cm=cm_snapshot))
        return True

    def on_local_costmap(self, msg):
        t = ns(msg.header.stamp)*1e-9
        obstacle = self.match_obstacle(t)
        if obstacle is None:
            return
        if msg.header.frame_id != 'odom':
            raise ValueError('expected local costmap in odom')
        ox, oy, res = msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution
        width, height = msg.info.width, msg.info.height
        def value(x, y):
            ix, iy = int(math.floor((x-ox)/res)), int(math.floor((y-oy)/res))
            if 0 <= ix < width and 0 <= iy < height:
                return int(msg.data[iy*width+ix])
            return None
        x, y, _ = obstacle['position']; sx, sy, _ = obstacle['size']
        # Inspect box boundary samples, where LaserScan marks actual surfaces.
        values = [value(x+dx, y+dy) for dx in (-sx/2-.05, -sx/2, 0, sx/2, sx/2+.05)
                  for dy in (-sy/2-.05, -sy/2, 0, sy/2, sy/2+.05)]
        observed = [v for v in values if v is not None]
        old_x = {'crossing_slot': 2.75, 'blocking_slot': 4.5, 'approach_slot': 6.0}[obstacle['slot']]
        self.cost_samples.append(dict(sim=t, obstacle=obstacle, frame=msg.header.frame_id,
                                      origin=[ox, oy], resolution=res, width=width, height=height,
                                      boundary_values=values, max_cost=max(observed) if observed else None,
                                      vacated_center_value=value(old_x, 0.),
                                      final_cmd=self.cmds['final'], plan_count=len(self.plans)))

    def graph(self, topics):
        return {t: dict(publishers=[i.node_name for i in self.get_publishers_info_by_topic(t)],
                        subscribers=[i.node_name for i in self.get_subscriptions_info_by_topic(t)],
                        types=sorted({i.topic_type for i in self.get_publishers_info_by_topic(t)}))
                for t in topics}


class Session:
    """One discovery server shared by consecutive runs of a script."""

    def __init__(self, label):
        assert os.environ.get('T1_LOCAL_DDS') == '1', 'source scripts/env.sh with T1_LOCAL_DDS=1'
        self.log = (RAW/f'{label}-dds.txt').open('w')
        self.server = subprocess.Popen([str(WS/'scripts/start_local_dds.sh')], stdout=self.log, stderr=subprocess.STDOUT)
        time.sleep(.5)
        try:
            assert self.server.poll() is None, 'discovery server failed to start'
            rclpy.init()
        except BaseException:
            if self.server.poll() is None:
                self.server.send_signal(signal.SIGINT)
                self.server.wait(timeout=10)
            self.log.close()
            raise

    def close(self):
        if rclpy.ok():
            rclpy.shutdown()
        self.server.send_signal(signal.SIGINT)
        self.server.wait(timeout=10)
        self.log.close()


class Run:
    def __init__(self, launch, label, args=(), wall_limit=400, authority=True):
        self.run_id = os.environ.get('T1_EVAL_RUN_ID') or f'{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_dev_{label}_s0_{uuid.uuid4().hex[:6]}'
        self.node = Probe()
        self.command = ['ros2', 'launch', 't1_bringup', launch, f'run_id:={self.run_id}', f'target_rtf:={os.environ.get("T1_EVAL_RTF", "1.0")}', *args]
        self.log_path = RAW/(self.run_id + '.txt')
        self.log = self.log_path.open('w')
        self.proc = subprocess.Popen(self.command, cwd=WS, stdout=self.log, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, start_new_session=True)
        self.authority_path = RAW/(self.run_id + '-authority.jsonl')
        self.authority = None
        if authority:
            self.authority_log = self.authority_path.open('w')
            self.authority = subprocess.Popen([str(AUTHORITY)], stdout=self.authority_log, stderr=subprocess.DEVNULL)
        self.wall0, self.wall_limit = time.monotonic(), wall_limit
        self.started = datetime.now(timezone.utc).isoformat()
        self.source, self.next_pub = None, 0.
        self.children = []
        self.cpu_samples, self.cpu_previous = [], {}
        self.next_cpu = self.wall0
        self.closed_result = None
        self.document_hash = digest(WS/'docs/start_commands.md')
        self.owned_processes = {}
        ACTIVE_RUNS.append(self)
        print('RUN', self.run_id, 'PID', self.proc.pid, flush=True)

    def spin(self):
        assert self.proc.poll() is None, f'launch exited; see {self.log_path}'
        assert time.monotonic() - self.wall0 < self.wall_limit, 'overall wall deadline'
        rclpy.spin_once(self.node, timeout_sec=.004)
        if time.monotonic() >= self.next_cpu:
            self.sample_cpu()
        if self.source is not None and time.monotonic() >= self.next_pub:
            if self.node.teleop is None:
                self.node.teleop = self.node.create_publisher(Twist, '/t1/cmd_teleop', 1)
            m = Twist()
            m.linear.x, m.angular.z = self.source
            self.node.teleop.publish(m)
            self.next_pub = time.monotonic() + .02

    def sample_cpu(self):
        now = time.monotonic()
        self.next_cpu = now + .5
        pending = [self.proc.pid]
        ticks = os.sysconf('SC_CLK_TCK')
        while pending:
            pid = pending.pop()
            try:
                root = Path('/proc')/str(pid)
                pending.extend(int(p) for p in (root/'task'/str(pid)/'children').read_text().split())
                command = (root/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
                stat = (root/'stat').read_text().rsplit(')', 1)[1].split()
                cpu = (int(stat[11])+int(stat[12]))/ticks
                self.owned_processes[pid] = stat[19]  # /proc starttime, guards PID reuse
                category = next((name for name in ('controller_server', 'planner_server', 'bt_navigator',
                    'behavior_server', 'amcl', 'collision_monitor', 't1_sim', 'rviz2') if name in command), None)
                previous = self.cpu_previous.get(pid)
                if category and previous:
                    elapsed = now-previous[0]
                    self.cpu_samples.append(dict(sim=self.node.sim, wall_interval_s=elapsed, pid=pid,
                        process=category, cpu_ms=(cpu-previous[1])*1000,
                        cpu_ms_per_wall_s=(cpu-previous[1])*1000/elapsed))
                self.cpu_previous[pid] = (now, cpu)
            except (OSError, ValueError, IndexError):
                pass  # process can exit between reads

    def until(self, predicate, timeout=30.):
        end = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < end, f'condition deadline; gate={self.node.gate}'
            self.spin()

    def duration(self, seconds):
        end = self.node.sim + seconds
        self.until(lambda: self.node.sim >= end, seconds*2 + 15)

    def call(self, kind, name, request, timeout=15.):
        client = self.node.create_client(kind, name)
        self.until(client.service_is_ready, timeout)
        future = client.call_async(request)
        self.until(future.done, timeout)
        self.node.destroy_client(client)
        return future.result()

    def ready(self):
        self.until(lambda: self.node.state and self.node.state['state'] == 'READY' and self.node.odom and self.node.scan, 60)

    def arm_teleop(self):
        self.source = (0., 0.)
        self.until(lambda: self.node.gate and not self.node.gate['health_reason'], 60)
        self.duration(.7)
        result = self.call(Trigger, '/t1/safety/arm', Trigger.Request())
        assert result.success, result.message
        self.until(lambda: self.node.gate['state'] == 'ARMED')

    def start_child(self, command, label):
        log = (RAW/f'{self.run_id}-{label}.txt').open('w')
        proc = subprocess.Popen(command, cwd=WS, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append((label, proc, log))
        return proc

    def screenshot(self, path):
        self.duration(.5)
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        windows = [line.split()[0] for line in tree.splitlines() if '"' in line and 'RViz' in line and 'rviz2' in line.lower()]
        assert windows, 'RViz window missing'
        if os.environ.get('T1_EVAL_HEADLESS') == 'false':
            viewer_windows = [line.split()[0] for line in tree.splitlines() if 'MuJoCo' in line]
            assert viewer_windows, 'MuJoCo viewer window missing'
            subprocess.run(['import', '-window', viewer_windows[-1], str(path.with_name(path.stem+'-mujoco.png'))], check=True, timeout=15)
        subprocess.run(['import', '-window', windows[-1], str(path)], check=True, timeout=15)

    def close(self, settle=2.):
        if self.closed_result is not None:
            return self.closed_result
        graph = None
        try:
            graph = self.node.graph(('/clock', '/cmd_vel', '/cmd_vel_nav', '/cmd_vel_smoothed', '/t1/cmd_navigation',
                                     '/t1/cmd_teleop', '/t1/nav/cmd_vel_raw', '/t1/eval/ground_truth',
                                     '/t1/debug/policy_cmd', '/map', '/scan', '/odom', '/initialpose', '/t1/eval/obstacle', '/local_costmap/costmap'))
        except Exception as exc:
            graph = dict(error=repr(exc))
        if self.proc.poll() is None:
            try:
                if self.source is not None:
                    self.source = (0., 0.)
                self.duration(settle)
            except Exception:
                pass
            self.proc.send_signal(signal.SIGINT)  # launch forwards once; group SIGINT double-interrupts children
        try:
            code = self.proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGTERM)
            code = self.proc.wait(timeout=15)
        for label, proc, log in self.children:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
            log.close()
        self.log.close()
        records = []
        if self.authority is not None:
            self.authority.send_signal(signal.SIGINT)
            self.authority.wait(timeout=10)
            self.authority_log.close()
            records = [json.loads(line) for line in self.authority_path.read_text().splitlines() if line.strip()]
        from .metrics import resolve_tf_authorities
        owners, gids = resolve_tf_authorities(records)
        manifest_path = WS/'run_data'/self.run_id/'manifest.json'
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        result = dict(run_id=self.run_id, command=self.command, cwd=str(WS), started_utc=self.started,
                      ended_utc=datetime.now(timezone.utc).isoformat(), exit_code=code,
                      wall_s=time.monotonic() - self.wall0, sim_end=self.node.sim, graph=graph,
                      tf_owners=owners, tf_gids=gids,
                      first_clock=self.node.first_clock, clock_regressions=self.node.clock_regressions, scan_stats=self.node.scan_stats,
                      initializations=self.node.initializations, events=self.node.events,
                      children=[(label, proc.returncode) for label, proc, _ in self.children],
                      manifest=manifest)
        (OUT/f'{self.run_id}-cpu.json').write_text(json.dumps(self.cpu_samples))
        result['documentation_sha256_at_start'] = self.document_hash
        result['recovery_counts_by_goal'] = self.node.recoveries
        result['remaining_owned_pids'] = []
        result['observed_owned_pids'] = sorted(self.owned_processes)
        for pid, starttime in self.owned_processes.items():
            try:
                stat = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()
                if stat[19] == starttime and stat[0] != 'Z':
                    result['remaining_owned_pids'].append(pid)
            except OSError:
                pass
        self.node.destroy_node()
        self.closed_result = result
        return result

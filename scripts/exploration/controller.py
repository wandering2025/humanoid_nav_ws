"""W16 decision process: online maps + TF + task offset, no simulator/truth input."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from t1_bringup.exploration import ObservedMap, transform_point
from t1_mujoco.exploration_task import load_task


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Explorer(Node):
    def __init__(self, task, output, limit):
        super().__init__('t1_explorer')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.task, self.out, self.limit = task, output, limit
        self.grid, self.costmap, self.gate, self.nav_handle = None, None, {}, None
        self.interrupted = False
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, qos)
        self.create_subscription(String, '/t1/safety/state', self.on_gate, qos)
        self.publisher = self.create_publisher(String, '/t1/exploration/status', qos)
        self.navigation = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.arm = self.create_client(Trigger, '/t1/safety/arm')
        self.stop = self.create_client(Trigger, '/t1/safety/stop')
        self.visited, self.failed = [], []
        self.sequence = 0
        self.events = (output/'decisions.jsonl').open('w')
        self.started_wall = time.monotonic()
        self.clock_value, self.clock_wall = 0., self.started_wall
        self.started_sim = None
        self.result = dict(status='STARTING', task=task, arrival_position_tolerance_m=.3)

    def sim(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_map(self, msg):
        if msg.header.frame_id != 'map':
            raise ValueError('exploration expects map frame')
        self.grid = msg

    def on_gate(self, msg):
        self.gate = json.loads(msg.data)

    def on_costmap(self, msg):
        if msg.header.frame_id != 'map':
            raise ValueError('costmap must use map frame')
        self.costmap = msg

    def pose(self, parent='map', child='base_footprint'):
        try:
            t = self.buffer.lookup_transform(parent, child, Time())
            if child == 'base_footprint' and self.sim()-(t.header.stamp.sec+t.header.stamp.nanosec*1e-9) > .5:
                return None
            return t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)
        except Exception:
            return None

    def target(self):
        correction = self.pose('map', 'odom')
        if correction is None:
            raise RuntimeError('map->odom unavailable')
        return transform_point(self.goal_odom, correction)

    def spin(self, check=True):
        rclpy.spin_once(self, timeout_sec=.02)
        now = time.monotonic()
        sim = self.sim()
        if sim != self.clock_value:
            if sim < self.clock_value:
                raise RuntimeError('clock regressed')
            self.clock_value, self.clock_wall = sim, now
        if check:
            if self.interrupted:
                raise InterruptedError('stop requested')
            if now-self.clock_wall > 3 and sim > 0:
                raise RuntimeError('clock stalled')
            if self.started_sim is not None and sim-self.started_sim > self.limit:
                raise TimeoutError('exploration simulation-time budget exhausted')
            if now-self.started_wall > self.limit*2.5+120:
                raise TimeoutError('exploration wall-time budget exhausted')
            if self.started_sim is not None and self.gate.get('state') != 'ARMED':
                raise RuntimeError(f'exploration safety latch: {self.gate}')

    def wait(self, predicate, seconds=10, check=True):
        end = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() > end:
                raise TimeoutError('ROS operation deadline')
            self.spin(check)

    def emit(self, event, **fields):
        data = dict(event=event, sim=self.sim(), **fields)
        line = json.dumps(data)
        self.events.write(line+'\n'); self.events.flush()
        self.publisher.publish(String(data=line))
        print(line, flush=True)

    def observed(self):
        m = self.costmap; p = m.info.origin
        return ObservedMap(np.asarray(m.data).reshape(m.info.height, m.info.width),
                           m.info.resolution, (p.position.x, p.position.y, yaw(p.orientation)),
                           self.pose()[:2], inflated=True)

    def snapshot(self, observed):
        self.sequence += 1
        name = f'map-{self.sequence:03d}.npz'
        np.savez_compressed(self.out/name, data=observed.data, resolution=observed.res,
                            origin=observed.origin, sim=self.sim(), inflated=True,
                            slam=np.asarray(self.grid.data).reshape(self.grid.info.height,self.grid.info.width),
                            slam_resolution=self.grid.info.resolution,
                            slam_origin=[self.grid.info.origin.position.x,self.grid.info.origin.position.y,
                                         yaw(self.grid.info.origin.orientation)])
        return name

    @staticmethod
    def stamped(target, heading=0.):
        from geometry_msgs.msg import PoseStamped
        p = PoseStamped(); p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y = map(float, target)
        p.pose.orientation.z = math.sin(heading/2); p.pose.orientation.w = math.cos(heading/2)
        return p

    def plan(self, target, observed):
        if not observed.is_reachable(target):
            return None
        goal = ComputePathToPose.Goal(); goal.planner_id = 'GridBased'
        goal.goal = self.stamped(target)
        sent = self.planner.send_goal_async(goal); self.wait(sent.done)
        handle = sent.result()
        if not handle.accepted:
            return None
        future = handle.get_result_async(); self.wait(future.done)
        reply = future.result()
        points = [(p.pose.position.x, p.pose.position.y) for p in reply.result.path.poses]
        if (reply.status != 4 or not points or math.dist(points[-1], target) > .15
                or not observed.path_known(points)):
            return None
        return points

    def send(self, target, path, final=False):
        reference = next((p for p in reversed(path) if math.dist(p, target) > 1.5), path[0])
        heading = math.atan2(target[1]-reference[1], target[0]-reference[0])
        goal = NavigateToPose.Goal(); goal.pose = self.stamped(target, heading)
        sent = self.navigation.send_goal_async(goal); self.wait(sent.done)
        self.nav_handle = sent.result()
        if not self.nav_handle.accepted:
            raise RuntimeError('navigation goal rejected')
        self.emit('navigate', target=list(target), heading=heading, final=final,
                  goal_id=bytes(self.nav_handle.goal_id.uuid).hex())
        return self.nav_handle.get_result_async()

    def latch_stop(self):
        if not self.stop.wait_for_service(timeout_sec=2):
            raise RuntimeError('stop service unavailable')
        reply = self.stop.call_async(Trigger.Request()); self.wait(reply.done, check=False)
        if not reply.result().success:
            raise RuntimeError('stop not acknowledged')
        if self.nav_handle:
            cancel = self.nav_handle.cancel_goal_async(); self.wait(cancel.done, check=False)

    def verify_arrival(self):
        self.latch_stop()
        samples = []; held = None; deadline = self.sim()+12
        wall_deadline = time.monotonic()+30
        while self.sim() < deadline and time.monotonic() < wall_deadline:
            self.spin(False)
            pose, now = self.pose(), self.sim()
            if pose is None or (samples and now-samples[-1][0] < .05):
                continue
            samples.append((now, pose))
            previous = [(t, p) for t, p in samples if t <= now-.5]
            if not previous:
                continue
            t, p = previous[-1]
            da = math.atan2(math.sin(pose[2]-p[2]), math.cos(pose[2]-p[2]))
            still = math.dist(pose[:2], p[:2])/(now-t) < .05 and abs(da)/(now-t) < .05
            held = (now if held is None else held) if still else None
            if held is not None and now-held >= 2:
                error = math.dist(pose[:2], self.target())
                self.result.update(position_error_m=error, actual_pose=pose,
                                   target_map=self.target(), stationary_s=now-held,
                                   arrival_sim=now)
                if error <= .3 and self.gate.get('state') == 'STOPPED':
                    return
                raise RuntimeError(f'stopped outside B tolerance: {error}')
        raise RuntimeError('arrival did not settle')

    def run(self):
        self.wait(lambda: self.sim() > 0 and self.grid is not None and self.costmap is not None and self.pose()
                  and self.pose('odom') and bool(self.gate) and self.navigation.server_is_ready()
                  and self.planner.server_is_ready(), 30)
        self.anchor = self.pose('odom')
        self.goal_odom = transform_point(self.task['offset'], self.anchor)
        self.result.update(anchor_odom=self.anchor, goal_odom=self.goal_odom)
        self.emit('start', anchor_odom=self.anchor, goal_odom=self.goal_odom)
        self.wait(lambda: self.arm.service_is_ready())
        future = self.arm.call_async(Trigger.Request()); self.wait(future.done)
        if not future.result().success:
            raise RuntimeError(future.result().message)
        self.wait(lambda: self.gate.get('state') == 'ARMED')
        self.started_sim = self.sim()
        for iteration in range(40):
            self.wait(lambda: self.pose() is not None)
            observed = self.observed(); target = self.target(); pose = self.pose()
            snapshot = self.snapshot(observed)
            path = self.plan(target, observed)
            final = path is not None
            candidates = []
            if not final:
                candidates = observed.candidates(pose[:2], target, self.visited, self.failed)
                for candidate in candidates[:8]:
                    path = self.plan(candidate['point'], observed)
                    candidate['nav2_valid'] = path is not None
                    if path:
                        target = candidate['point']; break
                if not path:
                    self.emit('exhausted', snapshot=snapshot, candidates=candidates)
                    raise RuntimeError('no reachable informative frontier; B route not established')
            self.emit('decision', iteration=iteration, snapshot=snapshot, pose=pose,
                      target=list(target), final=final, candidates=candidates, path=path)
            future = self.send(target, path, final)
            started = self.sim(); last_check = started; premature = 0
            while True:
                self.spin()
                here = self.pose()
                if here is None:
                    continue
                if final and math.dist(here[:2], self.target()) <= .15:
                    self.verify_arrival(); self.result['status'] = 'ARRIVED'; return
                if not final and math.dist(here[:2], target) <= .65:
                    self.visited.append(list(target)); self.emit('frontier_passed', target=list(target)); break
                if future.done():
                    status = future.result().status
                    self.emit('action_result', status=status, target=list(target), final=final)
                    if status == 4 and final and math.dist(here[:2], self.target()) <= .3:
                        self.verify_arrival(); self.result['status'] = 'ARRIVED'; return
                    if status == 4 and premature < 2:
                        premature += 1; future = self.send(target, path, final); continue
                    if status == 5:
                        raise InterruptedError('navigation canceled')
                    self.failed.append(list(target)); break
                if self.sim()-started > 120:
                    self.failed.append(list(target)); self.emit('frontier_timeout', target=list(target)); break
                if not final and self.sim()-last_check > 3:
                    last_check = self.sim()
                    fresh = self.observed()
                    direct = self.plan(self.target(), fresh)
                    if direct:
                        self.emit('B_route_discovered', snapshot=self.snapshot(fresh))
                        break  # next decision preempts without cancel/re-arm
        raise RuntimeError('exploration decision budget exhausted')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--time-limit', type=float, default=600.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    task = load_task(args.task)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Explorer(task, args.output, args.time_limit)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(node, 'interrupted', True))
    try:
        node.run()
    except Exception as exc:
        node.result.update(status='FAILED', error=repr(exc))
        node.emit('failure', error=repr(exc))
    finally:
        try:
            node.latch_stop()
        except Exception as exc:
            node.result.update(status='FAILED', stop_error=repr(exc))
        node.result.update(finished_sim=node.sim(), decisions=node.sequence)
        (args.output/'result.json').write_text(json.dumps(node.result, indent=2)+'\n')
        node.events.close(); node.destroy_node(); rclpy.shutdown()
    return 0 if node.result['status'] == 'ARRIVED' else 2


if __name__ == '__main__':
    raise SystemExit(main())

"""Single command source and latched wall-clock safety supervision (W10).

No physics/truth access. The permit is only a veto at the simulator boundary.
Normal commands must still traverse the actual Nav2 smoother and monitor.
"""
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import ParameterDescriptor


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SafetyGate(Node):
    def __init__(self):
        super().__init__('t1_command_gate')
        readonly = ParameterDescriptor(read_only=True)
        self.source = self.declare_parameter('command_source', 'teleop', readonly).value
        if self.source not in ('teleop', 'navigation'):
            raise ValueError('command_source must be teleop or navigation')
        self.timeout = float(self.declare_parameter('source_timeout', .3, readonly).value)
        self.health_timeout = float(self.declare_parameter('health_timeout', .3, readonly).value)
        if not all(math.isfinite(t) and 0 < t <= .3 for t in (self.timeout, self.health_timeout)):
            raise ValueError('timeouts must be positive and <= frozen 0.3 s safety budget')
        self.state, self.reason = 'DISARMED', 'explicit arm required'
        self.health = {}
        self.sim_state = 'STARTING'
        self.command = Twist()
        self.raw_at = None
        self.raw_zero = False
        self.final_zero = False
        self.last_nonzero_final = time.monotonic()
        self.active = {}
        self.last_poll = 0.
        self.graph_ok = False
        self.graph_at = 0.
        self.output = self.create_publisher(Twist, '/cmd_vel_nav', 1)
        self.permit = self.create_publisher(Bool, '/t1/safety/permit', 1)
        self.status = self.create_publisher(String, '/t1/safety/state',
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/t1/cmd_'+self.source, self.raw, 1)
        self.create_subscription(Twist, '/cmd_vel', self.final, 1)
        self.create_subscription(LaserScan, '/scan', self.scan, 1)
        self.create_subscription(Odometry, '/odom', self.odom, 1)
        self.create_subscription(TFMessage, '/tf', self.tf, 20)
        self.create_subscription(String, '/t1/sim_state', self.sim,
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/t1/safety/arm', self.arm)
        self.create_service(Trigger, '/t1/safety/stop', self.stop)
        self.lifecycle_clients = {name: self.create_client(GetState, '/'+name+'/get_state')
                        for name in ('velocity_smoother', 'collision_monitor')}
        self.pending = {}
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now_pair(self):
        return time.monotonic(), self.get_clock().now().nanoseconds*1e-9

    def mark(self, name, stamp):
        wall, sim = self.now_pair()
        # Never accept a stale or future stamped packet as a fresh heartbeat.
        if -.04 <= sim-stamp <= self.health_timeout:
            self.health[name] = (wall, stamp)

    def fresh(self, value, timeout):
        wall, sim = self.now_pair()
        return value is not None and 0 <= wall-value[0] <= timeout and -.04 <= sim-value[1] <= timeout

    def latch(self, reason):
        self.state, self.reason = 'STOPPED', reason
        self.command = Twist()
        self.permit.publish(Bool(data=False))
        self.output.publish(Twist())

    def raw(self, msg):
        values = (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.raw_at, self.raw_zero = None, False
            self.latch('INVALID_COMMAND')
            return
        self.raw_at = self.now_pair()
        self.raw_zero = all(v == 0 for v in values)
        if self.state == 'ARMED':
            # Clip only. The calibrated deadband/mapping remains in the bridge.
            self.command = Twist()
            self.command.linear.x = min(.75, max(0., msg.linear.x))
            self.command.angular.z = min(.85, max(-.85, msg.angular.z))
            if self.raw_zero:
                self.permit.publish(Bool(data=False))  # zero bypasses smoother ramp
        else:
            self.command = Twist()

    def final(self, msg):
        values = (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        self.final_zero = all(v == 0 for v in values)
        if not self.final_zero:
            self.last_nonzero_final = time.monotonic()

    def scan(self, msg):
        known = sum(not math.isnan(v) for v in msg.ranges)
        valid = (len(msg.ranges) == 360 and known >= .75*len(msg.ranges)
                 and not any(v == -math.inf for v in msg.ranges)
                 and msg.header.frame_id == 'laser_stabilized')
        if valid:
            self.mark('scan', seconds(msg.header.stamp))
        else:
            self.health.pop('scan', None)
            if self.state == 'ARMED':
                self.latch('SCAN_INVALID')

    def odom(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        if (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_footprint'
                and all(math.isfinite(x) for x in (p.x,p.y,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.angular.z))):
            self.mark('odom', seconds(msg.header.stamp))

    def tf(self, msg):
        for t in msg.transforms:
            edge = (t.header.frame_id, t.child_frame_id)
            if edge in (('odom','base_footprint'), ('base_footprint','laser_stabilized')):
                q, v = t.transform.rotation, t.transform.translation
                if all(math.isfinite(x) for x in (q.x,q.y,q.z,q.w,v.x,v.y,v.z)):
                    self.mark('tf:'+t.child_frame_id, seconds(t.header.stamp))

    def sim(self, msg):
        data = json.loads(msg.data)
        self.sim_state = data['state']
        if self.state == 'ARMED' and self.sim_state != 'READY':
            self.latch('SIM_'+self.sim_state)

    def health_reason(self):
        if self.sim_state != 'READY':
            return 'SIM_'+self.sim_state
        for name in ('scan','odom','tf:base_footprint','tf:laser_stabilized'):
            if not self.fresh(self.health.get(name), self.health_timeout):
                return 'STALE_'+name
        now = time.monotonic()
        for name in self.lifecycle_clients:
            if name not in self.active or now-self.active[name] > .6:
                return 'INACTIVE_'+name
        if not self.graph_ok or now-self.graph_at > .6:
            return 'COMMAND_OWNERSHIP'
        return ''

    def arm(self, _, response):
        reason = self.health_reason()
        if not reason and (not self.raw_zero or not self.fresh(self.raw_at, self.timeout)):
            reason = 'send fresh zero from selected source before arm'
        if not reason and (not self.final_zero or time.monotonic()-self.last_nonzero_final < .5):
            reason = 'wait for final filtered zero for 0.5 wall seconds'
        response.success = not reason
        if response.success:
            self.state, self.reason = 'ARMED', 'awaiting a new source command'
            self.command = Twist()
            # New source message required; pre-arm cached nonzero cannot revive.
        response.message = reason or self.reason
        return response

    def stop(self, _, response):
        self.latch('EXPLICIT_STOP_OR_CANCEL')
        response.success, response.message = True, self.reason
        return response

    def poll(self):
        now = time.monotonic()
        if now-self.last_poll < .1:
            return
        self.last_poll = now
        expected = {'/cmd_vel':'collision_monitor', '/cmd_vel_smoothed':'velocity_smoother',
                    '/cmd_vel_nav':'t1_command_gate'}
        self.graph_ok = all([i.node_name for i in self.get_publishers_info_by_topic(t)] == [n]
                            for t,n in expected.items())
        self.graph_ok &= len(self.get_publishers_info_by_topic('/t1/cmd_'+self.source)) == 1
        self.graph_at = now
        for name, client in self.lifecycle_clients.items():
            pending = self.pending.get(name)
            future = pending[1] if pending is not None else None
            if future is not None and not future.done() and now-pending[0] > .3:
                client.remove_pending_request(future)
                future.cancel()
                self.pending.pop(name, None)
                future = None
            if future is not None and future.done():
                try:
                    if future.result().current_state.id == 3:
                        self.active[name] = now
                    else:
                        self.active.pop(name, None)
                except Exception:
                    self.active.pop(name, None)
                self.pending.pop(name, None)
            if name not in self.pending and client.service_is_ready():
                self.pending[name] = (now, client.call_async(GetState.Request()))

    def tick(self):
        self.poll()
        if self.state == 'ARMED':
            reason = self.health_reason()
            if not reason and not self.fresh(self.raw_at, self.timeout):
                reason = 'SOURCE_TIMEOUT'
            if reason:
                self.latch(reason)
        enabled = self.state == 'ARMED'
        moving = self.command.linear.x != 0 or self.command.angular.z != 0
        self.permit.publish(Bool(data=enabled and moving))
        self.output.publish(self.command if enabled else Twist())
        self.status.publish(String(data=json.dumps(dict(state=self.state,reason=self.reason,
            source=self.source,permit=enabled and moving,wall_mono=time.monotonic(),
            sim_time=self.get_clock().now().nanoseconds*1e-9,health_reason=self.health_reason()))))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.latch('SHUTDOWN')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

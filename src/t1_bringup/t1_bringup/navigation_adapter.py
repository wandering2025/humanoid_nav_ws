"""Single Nav2 command source for the W10 gate (W11, ADR-016).

controller_server and behavior_server both publish /t1/nav/cmd_vel_raw (Nav2
design: the BT runs them serially). This node is the only publisher of
/t1/cmd_navigation and publishes continuously so the gate's source watchdog stays
meaningful: the latest fresh Nav2 command, otherwise zero.

Shaping happens before the smoother/collision monitor, never after it:
requests that W05 shows cannot move the gait are either zeroed or raised to the
smallest calibrated value. The bridge still only reduces, and the monitor's
stop/slowdown still applies to the shaped command. A canceled NavigateToPose
calls /t1/safety/stop (latched; a new goal needs explicit arm) and drops the
cached command so an old goal can never revive.
"""
import json
import math
import time
from pathlib import Path

import yaml
from .navigation_protocol import CommandEpoch

FINITE_FIELDS = ('vx', 'wz')


def load_envelope(path):
    data = yaml.safe_load(Path(path).read_text())
    if not data.get('calibrated'):
        raise ValueError('navigation adapter requires a calibrated W05 command_map')
    caps, env = data['capabilities'], data['envelope']
    if caps['reverse_allowed'] or caps['lateral_allowed']:
        raise ValueError('adapter assumes forward/yaw-only W05 capabilities')
    return dict(vx_min=env['vx_min'], vx_max=env['vx_max'], wz_walking_max=env['wz_walking_max'],
                wz_in_place_min=env['wz_in_place_min'], wz_in_place_max=env['wz_in_place_max'],
                in_place=caps['in_place_turn_allowed'])


def shape(vx, wz, env, vx_lift_threshold, vx_lift_target, wz_lift_threshold, wz_in_place_target,
          min_walking_radius=0.70):
    """Map a Nav2 (vx, wz) request into the W05 feasible set. Returns (vx, wz, reason).

    Below the gait minimum, the request's curvature decides: tighter than the walking
    minimum radius -> in-place turn, otherwise -> slowest calibrated walk.
    Non-finite input is passed through unchanged so the W10 gate latches it.
    """
    if not (math.isfinite(vx) and math.isfinite(wz)):
        return vx, wz, 'nonfinite_passthrough'
    reasons = []
    if vx < 0:
        vx, reasons = 0.0, reasons + ['reverse_zeroed']
    if vx >= env['vx_min']:
        vx = min(vx, env['vx_max'])
        if abs(wz) > env['wz_walking_max']:
            wz, reasons = math.copysign(env['wz_walking_max'], wz), reasons + ['wz_walking_clipped']
        return vx, wz, ','.join(reasons) or 'walk'
    turning = env['in_place'] and abs(wz) >= wz_lift_threshold
    if turning and vx < min_walking_radius*abs(wz):
        target = min(max(abs(wz), wz_in_place_target, env['wz_in_place_min']), env['wz_in_place_max'])
        return 0.0, math.copysign(target, wz), ','.join(reasons + ['in_place'])
    if vx >= vx_lift_threshold:
        vx = vx_lift_target
        if abs(wz) > env['wz_walking_max']:
            wz = math.copysign(env['wz_walking_max'], wz)
        return vx, wz, ','.join(reasons + ['vx_lifted'])
    # Effectively stationary request: in-place turn or full stop.
    if env['in_place'] and abs(wz) >= wz_lift_threshold:
        target = min(max(abs(wz), wz_in_place_target, env['wz_in_place_min']), env['wz_in_place_max'])
        return 0.0, math.copysign(target, wz), ','.join(reasons + ['in_place'])
    reason = 'stop' if vx == 0 and wz == 0 else 'below_gait_zeroed'
    return 0.0, 0.0, ','.join(reasons + [reason])


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from rcl_interfaces.msg import ParameterDescriptor

    class Adapter(Node):
        def __init__(self):
            super().__init__('t1_navigation_adapter')
            ro = ParameterDescriptor(read_only=True)
            get = lambda name, default: self.declare_parameter(name, default, ro).value
            self.env = load_envelope(get('command_map_file', ''))
            self.rate = float(get('publish_rate_hz', 50.0))
            self.timeout = float(get('input_timeout_s', 0.15))
            self.shaping = dict(vx_lift_threshold=float(get('vx_lift_threshold', .16)),
                                vx_lift_target=float(get('vx_lift_target', .33)),
                                wz_lift_threshold=float(get('wz_lift_threshold', .2)),
                                wz_in_place_target=float(get('wz_in_place_target', .6)),
                                min_walking_radius=float(get('min_walking_radius', .70)))
            s = self.shaping
            if not (0 < s['vx_lift_threshold'] < self.env['vx_min'] <= s['vx_lift_target'] <= self.env['vx_max']):
                raise ValueError('vx lift must raise sub-gait requests to [vx_min, vx_max]')
            if not (0 < s['wz_lift_threshold'] and
                    self.env['wz_in_place_min'] <= s['wz_in_place_target'] <= self.env['wz_in_place_max']):
                raise ValueError('in-place target must lie in the W05 in-place band')
            if not (0 < self.timeout <= .3 and 20 <= self.rate <= 100):
                raise ValueError('adapter must publish at >=20 Hz and expire input within the 0.3 s gate budget')
            self.epoch = CommandEpoch()
            self.out = (0.0, 0.0, 'idle')
            self.pub = self.create_publisher(Twist, '/t1/cmd_navigation', 1)
            self.state_pub = self.create_publisher(String, '/t1/nav/adapter_state', 10)
            self.create_subscription(Twist, '/t1/nav/cmd_vel_raw', self.input, 10)
            self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goals,
                                     QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                                     lambda m: self.goals(m, 'follow'),
                                     QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.stop_client = self.create_client(Trigger, '/t1/safety/stop')
            self.GoalStatus = GoalStatus
            self.Twist = Twist
            self.String = String
            self.create_timer(1/self.rate, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def input(self, msg):
            self.epoch.receive(msg.linear.x, msg.angular.z, time.monotonic())

        def goals(self, msg, kind='nav'):
            rows = [(bytes(s.goal_info.goal_id.uuid).hex(),
                     s.goal_info.stamp.sec*1_000_000_000 + s.goal_info.stamp.nanosec, s.status)
                    for s in msg.status_list]
            if self.epoch.statuses(kind, rows):
                if self.stop_client.service_is_ready():
                    self.stop_client.call_async(Trigger.Request())
                else:
                    self.get_logger().error('/t1/safety/stop unavailable; epoch blocked, publishing zero')

        def tick(self):
            now = time.monotonic()
            raw = self.epoch.command(now, self.timeout)
            if raw is None:
                vx, wz, reason = 0.0, 0.0, 'no_current_goal_command'
            else:
                vx, wz, reason = shape(*raw, self.env, **self.shaping)
            msg = self.Twist()
            msg.linear.x, msg.angular.z = float(vx), float(wz)
            self.pub.publish(msg)
            self.out = (vx, wz, reason)
            self.state_pub.publish(self.String(data=json.dumps(dict(
                raw=raw, out=[vx, wz], reason=reason, blocked_goal=self.epoch.nav[0] if self.epoch.nav and not self.epoch.allowed() else None,
                sim_time=self.get_clock().now().nanoseconds*1e-9, wall_mono=now), allow_nan=True)))

    rclpy.init(args=args)
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

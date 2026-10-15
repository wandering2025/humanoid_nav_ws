"""Event-driven readiness: graph discovery is only the first of three gates."""
import json
import time

from .navigation_protocol import ProtocolFailure, await_future, require_response

LOCALIZATION = ('map_server', 'amcl')
NAVIGATION = ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
              'local_costmap/local_costmap', 'global_costmap/global_costmap')


class Readiness:
    def __init__(self, node, *, spin_once=None):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from tf2_ros import Buffer, TransformListener
        from lifecycle_msgs.srv import GetState
        self.node, self.state, self.scan, self.initialized = node, None, None, False
        self._spin_once = spin_once
        self.first_clock = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/t1/sim_state', self.on_state, tl)
        node.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m.header.stamp), 5)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: setattr(self, 'initialized', True), tl)
        self.clients = {n: node.create_client(GetState, '/'+n+'/get_state') for n in LOCALIZATION+NAVIGATION}
        self.trace = []

    def on_state(self, msg):
        self.state = json.loads(msg.data)['state']

    def spin(self):
        if self._spin_once is not None:
            return self._spin_once()
        import rclpy
        if not rclpy.ok():
            raise ProtocolFailure('CONTEXT_SHUTDOWN')
        rclpy.spin_once(self.node, timeout_sec=.02)

    def sensors(self):
        from rclpy.time import Time
        now = self.node.get_clock().now().nanoseconds*1e-9
        if self.first_clock is None and now > 0:
            self.first_clock = now
        age = None if self.scan is None else now-(self.scan.sec+self.scan.nanosec*1e-9)
        return dict(sim_ready=self.state == 'READY',
                    clock_advancing=self.first_clock is not None and now > self.first_clock+.5,
                    scan_fresh=age is not None and -.05 <= age <= .3,
                    initialpose_ack=self.initialized,
                    map_odom=self.buffer.can_transform('map', 'odom', Time()),
                    localized=self.buffer.can_transform('map', 'base_footprint', Time()))

    def wait_sensors(self, deadline):
        checks = self.sensors()
        while time.monotonic() < deadline and not all(checks.values()):
            self.spin(); checks = self.sensors()
        if not all(checks.values()):
            raise ProtocolFailure('READINESS_MISSING:' + ','.join(k for k, v in checks.items() if not v))

    def lifecycle(self, names, deadline):
        from lifecycle_msgs.srv import GetState
        for name in names:
            last = 'NOT_ACTIVE'
            for attempt in range(3):
                if time.monotonic() >= deadline:
                    break
                client = self.clients[name]
                if not client.wait_for_service(timeout_sec=min(2., max(0., deadline-time.monotonic()))):
                    last = 'NOT_DISCOVERED'; continue
                try:
                    response = require_response(client, GetState.Request(), self.spin, 2., 'LIFECYCLE_'+name)
                    if response.current_state.id == 3:
                        self.trace.append(dict(node=name, state=3, attempt=attempt+1)); break
                    last = 'NOT_ACTIVE:'+str(response.current_state.id)
                except ProtocolFailure as exc:
                    last = str(exc)
            else:
                raise ProtocolFailure(name+':'+last)
            if time.monotonic() >= deadline:
                raise ProtocolFailure(name+':DEADLINE')
            if not self.trace or self.trace[-1].get('node') != name:
                raise ProtocolFailure(name+':'+last)

    def endpoints(self):
        from rclpy.action import ActionClient
        from nav2_msgs.action import FollowPath, NavigateToPose, ComputePathToPose
        from nav2_msgs.srv import ClearEntireCostmap
        for kind, name in ((NavigateToPose, 'navigate_to_pose'), (ComputePathToPose, 'compute_path_to_pose'),
                           (FollowPath, 'follow_path')):
            action = ActionClient(self.node, kind, name)
            try:
                if not action.wait_for_server(timeout_sec=2.):
                    raise ProtocolFailure('ACTION_NOT_DISCOVERED:'+name)
                if name == 'follow_path':
                    # Empty path is accepted then rejected as INVALID_PATH by the active
                    # controller. It commands no motion; W10 is still DISARMED. This
                    # tests the real ACK/result path, not just graph visibility.
                    handle = await_future(action.send_goal_async(FollowPath.Goal()), self.spin, 2., 'FOLLOW_PATH_ACK')
                    if not handle.accepted:
                        raise ProtocolFailure('FOLLOW_PATH_REJECTED')
                    result = await_future(handle.get_result_async(), self.spin, 2., 'FOLLOW_PATH_PROBE_RESULT')
                    if result.status != 6 or result.result.error_code != 103:
                        raise ProtocolFailure('FOLLOW_PATH_PROBE_UNEXPECTED_RESULT')
                    self.trace.append(dict(action=name, ack=True, expected_invalid_path=103))
            finally:
                action.destroy()
        for name in ('local_costmap/clear_entirely_local_costmap', 'global_costmap/clear_entirely_global_costmap'):
            client = self.node.create_client(ClearEntireCostmap, '/'+name)
            try:
                if not client.wait_for_service(timeout_sec=2.):
                    raise ProtocolFailure('COSTMAP_NOT_DISCOVERED:'+name)
                require_response(client, ClearEntireCostmap.Request(), self.spin, 2., 'COSTMAP_RESPONSE:'+name)
                self.trace.append(dict(service=name, responded=True))
            finally:
                self.node.destroy_client(client)

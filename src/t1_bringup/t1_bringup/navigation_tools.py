"""W11 navigation readiness starter and NavigateToPose goal tool. No truth access.

navigation_starter: waits until the simulator is READY, /clock advances, a fresh
/scan arrives and map->base_footprint resolves (AMCL initialized), then asks the
navigation lifecycle manager to STARTUP the Nav2 servers. Nothing is activated
before localization exists.

navigate_to_pose X Y YAW: waits for bt_navigator active, explicitly arms the W10
gate (zero must already be flowing), sends one NavigateToPose goal in the map
frame, enforces a simulation-time limit and cancels on expiry. Exit codes:
0 SUCCEEDED, 2 ABORTED/CANCELED, 3 time limit (canceled), 1 setup failure.
"""
import argparse
import json
import math
import time


def _yaw_quaternion(yaw):
    return math.sin(yaw/2), math.cos(yaw/2)


def starter(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from nav2_msgs.srv import ManageLifecycleNodes
    from tf2_ros import Buffer, TransformListener

    rclpy.init(args=args)
    node = Node('t1_navigation_starter')
    timeout = node.declare_parameter('ready_timeout_wall_s', 120.0).value
    manager = node.declare_parameter('lifecycle_manager', 'lifecycle_manager_navigation').value
    buffer = Buffer()
    TransformListener(buffer, node)
    seen = dict(state=None, scan=None)
    node.create_subscription(String, '/t1/sim_state', lambda m: seen.update(state=json.loads(m.data)['state']),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(LaserScan, '/scan', lambda m: seen.update(scan=m.header.stamp), 5)
    client = node.create_client(ManageLifecycleNodes, f'/{manager}/manage_nodes')
    deadline = time.monotonic() + timeout
    code, reason, first_clock = 1, 'timeout', None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now = node.get_clock().now().nanoseconds*1e-9
            first_clock = first_clock if first_clock is not None or now <= 0 else now
            scan_age = None if seen['scan'] is None else now - (seen['scan'].sec + seen['scan'].nanosec*1e-9)
            checks = dict(sim_ready=seen['state'] == 'READY',
                          clock_advancing=first_clock is not None and now > first_clock + .5,
                          scan_fresh=scan_age is not None and -.05 <= scan_age <= .3,
                          localized=buffer.can_transform('map', 'base_footprint', Time()),
                          manager=client.service_is_ready())
            if all(checks.values()):
                request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
                future = client.call_async(request)
                end = time.monotonic() + 60
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.1)
                ok = future.done() and future.result().success
                code, reason = (0, 'navigation active') if ok else (1, 'lifecycle STARTUP failed')
                break
            reason = 'waiting: ' + ','.join(k for k, v in checks.items() if not v)
        print(json.dumps(dict(starter=reason, exit=code, sim_time=node.get_clock().now().nanoseconds*1e-9)),
              flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


def navigate(argv=None):
    parser = argparse.ArgumentParser(description='Send one NavigateToPose goal (map frame) through the W10 gate.')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw', type=float, help='radians in map')
    parser.add_argument('--time-limit', type=float, default=120.0, help='simulation seconds from acceptance')
    parser.add_argument('--setup-timeout', type=float, default=180.0, help='wall seconds for nav/arm readiness')
    parser.add_argument('--no-arm', action='store_true', help='require the gate to be ARMED already')
    options = parser.parse_args(argv)
    if not all(math.isfinite(v) for v in (options.x, options.y, options.yaw, options.time_limit)):
        parser.error('goal and limit must be finite')

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus

    rclpy.init()
    node = Node('t1_navigate_to_pose')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    gate = {}
    node.create_subscription(String, '/t1/safety/state', lambda m: gate.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
    arm = node.create_client(Trigger, '/t1/safety/arm')
    action = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    summary = dict(goal=[options.x, options.y, options.yaw], time_limit_s=options.time_limit)
    code = 1

    def spin_until(predicate, wall_timeout):
        end = time.monotonic() + wall_timeout
        while not predicate():
            if time.monotonic() > end:
                return False
            rclpy.spin_once(node, timeout_sec=.05)
        return True

    def sim():
        return node.get_clock().now().nanoseconds*1e-9

    try:
        setup_end = time.monotonic() + options.setup_timeout
        active = False
        while not active and time.monotonic() < setup_end:
            if lifecycle.wait_for_service(timeout_sec=.5):
                future = lifecycle.call_async(GetState.Request())
                spin_until(future.done, 2)
                active = future.done() and future.result().current_state.id == 3
            if not active:
                spin_until(lambda: False, .5)
        if not active:
            raise RuntimeError('bt_navigator is not active (navigation_starter not finished?)')
        if not spin_until(lambda: bool(gate), 10):
            raise RuntimeError('no /t1/safety/state from t1_command_gate')
        if gate.get('source') != 'navigation':
            raise RuntimeError(f"gate source is {gate.get('source')!r}, expected navigation")
        arm_messages = []
        while gate.get('state') != 'ARMED':
            if options.no_arm:
                raise RuntimeError(f"gate not ARMED: {gate.get('state')} {gate.get('reason')}")
            if time.monotonic() > setup_end:
                raise RuntimeError(f'arm failed: {arm_messages[-3:]}')
            if arm.wait_for_service(timeout_sec=.5):
                future = arm.call_async(Trigger.Request())
                spin_until(future.done, 2)
                if future.done():
                    arm_messages.append(future.result().message)
            spin_until(lambda: gate.get('state') == 'ARMED', .5)
        summary['arm_attempts'] = len(arm_messages)
        if not action.wait_for_server(timeout_sec=10):
            raise RuntimeError('navigate_to_pose action server unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = options.x, options.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = _yaw_quaternion(options.yaw)
        feedback = {}

        def on_feedback(msg):
            f = msg.feedback
            feedback.update(distance_remaining=f.distance_remaining, recoveries=f.number_of_recoveries,
                            navigation_time=f.navigation_time.sec + f.navigation_time.nanosec*1e-9)
        send = action.send_goal_async(goal, feedback_callback=on_feedback)
        if not spin_until(send.done, 10) or not send.result().accepted:
            raise RuntimeError('goal rejected')
        handle = send.result()
        start = sim()
        summary.update(accepted_sim=start, goal_id=bytes(handle.goal_id.uuid).hex())
        print(json.dumps(dict(event='accepted', **summary)), flush=True)
        result = handle.get_result_async()
        timed_out = False
        while not result.done():
            rclpy.spin_once(node, timeout_sec=.05)
            if sim() - start > options.time_limit:
                timed_out = True
                cancel = handle.cancel_goal_async()
                spin_until(cancel.done, 10)
                spin_until(result.done, 15)
                break
        status = result.result().status if result.done() else None
        names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                 GoalStatus.STATUS_CANCELED: 'CANCELED'}
        summary.update(status=names.get(status, str(status)), timed_out=timed_out, end_sim=sim(),
                       duration_sim=sim()-start, feedback=feedback,
                       error_code=getattr(result.result().result, 'error_code', None) if result.done() else None)
        code = 3 if timed_out else (0 if status == GoalStatus.STATUS_SUCCEEDED else 2)
    except Exception as exc:
        summary.update(status='SETUP_FAILED', error=repr(exc))
    finally:
        summary['exit'] = code
        print(json.dumps(dict(event='result', **summary)), flush=True)
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return code


def starter_main():
    raise SystemExit(starter())


def navigate_main():
    raise SystemExit(navigate())

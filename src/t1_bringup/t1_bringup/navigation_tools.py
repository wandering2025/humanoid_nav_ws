"""W11 navigation readiness starter and NavigateToPose goal tool. No truth access.

navigation_starter: waits until the simulator is READY, /clock advances, a fresh
/scan arrives and map->base_footprint resolves (AMCL initialized), then asks the
navigation lifecycle manager to STARTUP the Nav2 servers. Nothing is activated
before localization exists.

navigate_to_pose X Y YAW: waits for bt_navigator active, explicitly arms the W10
gate (zero must already be flowing), sends a NavigateToPose goal in the map
frame and enforces one simulation-time limit for the whole task (cancel on expiry).

Terminal settle-and-verify (W11 C2): Nav2's goal checkers test instantaneous
velocity, and the T1 gait yaw oscillation crosses zero while the body is still
rebounding (measured 17-19 deg after a 170 deg in-place turn). After SUCCEEDED the
tool therefore waits until the localized pose (map->base_footprint, i.e. AMCL +
odom; never truth) is stationary over a window, and re-sends the same goal for a
short correction if position/yaw are outside tolerance (bounded retries). Exit
codes: 0 verified SUCCEEDED, 2 ABORTED/CANCELED/unverified, 3 time limit, 1 setup.
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
    parser.add_argument('--xy-tolerance', type=float, default=0.20, help='verified map position tolerance (m)')
    parser.add_argument('--yaw-tolerance', type=float, default=0.20, help='verified map yaw tolerance (rad)')
    parser.add_argument('--terminal-retries', type=int, default=3)
    parser.add_argument('--approach-distance', type=float, default=1.5,
                        help='m; walk the last segment along the goal heading (0 disables)')
    parser.add_argument('--retry-gain', type=float, default=1.0,
                        help='terminal retry aims past the goal yaw by gain*residual (clamped 0.5 rad)')
    parser.add_argument('--approach-pass-radius', type=float, default=0.35,
                        help='m; preempt the staging waypoint with the goal once this close')
    parser.add_argument('--settle-window', type=float, default=0.5, help='sim s; W05 stationary window')
    parser.add_argument('--settle-hold', type=float, default=1.0, help='sim s stationary before verifying')
    parser.add_argument('--settle-timeout', type=float, default=10.0, help='sim s to become stationary')
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
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from nav_msgs.msg import OccupancyGrid

    rclpy.init()
    node = Node('t1_navigate_to_pose')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    gate, costmap = {}, {}
    node.create_subscription(String, '/t1/safety/state', lambda m: gate.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    # Global costmap (static SLAM map + scan obstacles + 2.05 m inflation), only to
    # choose a feasible staging distance. OccupancyGrid 99/100 = inscribed/lethal.
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: costmap.update(grid=m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
    arm = node.create_client(Trigger, '/t1/safety/arm')
    action = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    buffer = Buffer()
    TransformListener(buffer, node)
    summary = dict(goal=[options.x, options.y, options.yaw], time_limit_s=options.time_limit,
                   tolerances=dict(xy=options.xy_tolerance, yaw=options.yaw_tolerance), attempts=[])
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
        names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                 GoalStatus.STATUS_CANCELED: 'CANCELED'}
        target = (options.x, options.y, options.yaw)
        state = dict(start=None, timed_out=False)

        def make_goal(x, y, a):
            msg = NavigateToPose.Goal()
            msg.pose.header.frame_id = 'map'
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = _yaw_quaternion(a)
            return msg

        def map_pose():
            try:
                t = buffer.lookup_transform('map', 'base_footprint', Time())
            except Exception:
                return None
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)))

        def expired():
            return state['start'] is not None and sim() - state['start'] > options.time_limit

        def settle():
            """Wait for a stationary localized pose (W05 window criterion held for
            settle_hold); return (pose, (xy_error, yaw_error)) or None."""
            samples, end, since = [], sim() + options.settle_timeout, None
            while sim() < end and not expired():
                rclpy.spin_once(node, timeout_sec=.02)
                pose, now = map_pose(), sim()
                if pose is None or (samples and now - samples[-1][0] < .05):
                    continue
                samples.append((now, pose))
                ref = [q for u, q in samples if u <= now - options.settle_window]
                if not ref:
                    continue
                o = ref[-1]
                dyaw = math.atan2(math.sin(pose[2]-o[2]), math.cos(pose[2]-o[2]))
                still = (math.dist(pose[:2], o[:2])/options.settle_window < .05 and
                         abs(dyaw)/options.settle_window < .05)
                since = (since if since is not None else now) if still else None
                if since is not None and now - since >= options.settle_hold:
                    x, y, a = pose
                    return pose, (math.hypot(x-target[0], y-target[1]),
                                  abs(math.atan2(math.sin(a-target[2]), math.cos(a-target[2]))))
            return None

        def execute(stage, pose, pass_radius=None):
            send = action.send_goal_async(make_goal(*pose))
            if not spin_until(send.done, 10) or not send.result().accepted:
                raise RuntimeError(f'{stage} goal rejected')
            handle = send.result()
            if state['start'] is None:
                state['start'] = sim()
                summary.update(accepted_sim=state['start'], goal_id=bytes(handle.goal_id.uuid).hex())
                print(json.dumps(dict(event='accepted', **summary)), flush=True)
            record = dict(stage=stage, target=list(pose), accepted_sim=sim(), goal_id=bytes(handle.goal_id.uuid).hex())
            result = handle.get_result_async()
            while not result.done():
                rclpy.spin_once(node, timeout_sec=.05)
                here = map_pose() if pass_radius else None
                if here is not None and math.dist(here[:2], pose[:2]) <= pass_radius:
                    # Pass-through waypoint: the caller preempts with the next goal
                    # (no cancel, so the W10 stop latch is not involved).
                    record.update(status='PASSED', end_sim=sim(), passed_pose=list(here))
                    summary['attempts'].append(record)
                    print(json.dumps(dict(event='stage', **record)), flush=True)
                    return record
                if expired():
                    state['timed_out'] = True
                    cancel = handle.cancel_goal_async()
                    spin_until(cancel.done, 10)
                    spin_until(result.done, 15)
                    break
            status = result.result().status if result.done() else None
            record.update(status=names.get(status, str(status)), end_sim=sim(),
                          error_code=getattr(result.result().result, 'error_code', None) if result.done() else None)
            summary['attempts'].append(record)
            print(json.dumps(dict(event='stage', **record)), flush=True)
            return record

        # Staged final approach (W11 C2): the T1 stance rebounds 0.1-0.4 rad against
        # the direction of any in-place turn, so repeated in-place corrections land
        # short; walking stops rebound <=5.2 deg (W05). Walk the last approach_distance
        # along the goal heading from a pre-goal, unless already that close.
        def segment_free(dist):
            grid = costmap.get('grid')
            if grid is None:
                return False
            info = grid.info
            for k in range(int(dist/.05) + 1):
                x = target[0] - k*.05*math.cos(target[2])
                y = target[1] - k*.05*math.sin(target[2])
                i = int((x - info.origin.position.x)/info.resolution)
                j = int((y - info.origin.position.y)/info.resolution)
                if not (0 <= i < info.width and 0 <= j < info.height):
                    return False
                value = grid.data[j*info.width + i]
                if value < 0 or value >= 99:
                    return False
            return True

        spin_until(lambda: 'grid' in costmap, 5)
        here = map_pose()
        candidates = [d for d in (options.approach_distance, .75*options.approach_distance, .5*options.approach_distance)
                      if d > 0 and (here is None or math.dist(here[:2], target[:2]) > d + options.xy_tolerance)]
        chosen = next((d for d in candidates if segment_free(d)), None)
        summary['approach_distance'] = chosen
        if chosen:
            pre = (target[0] - chosen*math.cos(target[2]), target[1] - chosen*math.sin(target[2]), target[2])
            execute('approach', pre, options.approach_pass_radius)  # never counts as arrival
        verified, aim = None, target
        if not state['timed_out']:
            for attempt in range(options.terminal_retries + 1):
                record = execute('goal' if attempt == 0 else 'terminal_retry', aim)
                if state['timed_out'] or record['status'] != 'SUCCEEDED':
                    break
                settled = settle()
                if settled is None:
                    record['verify'] = 'not_stationary'
                    if expired():
                        state['timed_out'] = True
                        break
                    continue
                pose, (exy, eyaw) = settled
                record.update(verify_sim=sim(), verified_pose=list(pose), xy_error=exy, yaw_error=eyaw)
                if exy <= options.xy_tolerance and eyaw <= options.yaw_tolerance:
                    verified = record
                    break
                # Integral correction: an in-place turn lands short by about its
                # rebound, so aim past the target by the measured residual.
                signed = math.atan2(math.sin(target[2]-pose[2]), math.cos(target[2]-pose[2]))
                aim = (target[0], target[1], target[2] + max(-.5, min(.5, options.retry_gain*signed)))
                record['next_aim_yaw'] = aim[2]
                print(json.dumps(dict(event='terminal_retry', **record)), flush=True)
        goal_attempts = [a for a in summary['attempts'] if a['stage'] != 'approach']
        last = summary['attempts'][-1]
        summary.update(status='SUCCEEDED' if verified else (last['status'] if last['status'] != 'SUCCEEDED' else 'UNVERIFIED'),
                       timed_out=state['timed_out'], end_sim=sim(), duration_sim=sim()-state['start'],
                       verified=verified, terminal_retries=max(0, len(goal_attempts)-1), error_code=last.get('error_code'),
                       nav2_success_sim=next((a['end_sim'] for a in reversed(goal_attempts) if a['status'] == 'SUCCEEDED'), None))
        code = 3 if state['timed_out'] else (0 if verified else 2)
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

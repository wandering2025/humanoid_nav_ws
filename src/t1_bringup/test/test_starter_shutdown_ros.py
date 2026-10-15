"""Actual starter subprocess + ROS endpoints; no simulation or velocity source.

The client barrier places one real OS SIGINT inside the original failing call
chain, after Node has removed the client from its list. It does not change the
starter or rclpy cleanup. Full cold-launch coverage lives in W11R2 evidence.
"""
import json
import os
import signal
import subprocess
import time

import pytest
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath, NavigateToPose, ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from test_bounded_manager_ros import manager  # isolated DDS + spinning service node
from t1_bringup.navigation_readiness import LOCALIZATION, NAVIGATION
from t1_bringup.starter_runtime import StarterRuntime


@pytest.fixture
def ready_endpoints(manager):
    node, _ = manager
    group = ReentrantCallbackGroup()

    def active(req, res):
        res.current_state.id = 3
        return res

    def startup(req, res):
        res.success = True
        return res

    for name in LOCALIZATION + NAVIGATION:
        node.create_service(GetState, '/'+name+'/get_state', active, callback_group=group)
    node.create_service(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes', startup)
    for name in ('local_costmap', 'global_costmap'):
        node.create_service(ClearEntireCostmap, f'/{name}/clear_entirely_{name}', lambda req, res: res)

    def invalid_path(goal):
        goal.abort()
        return FollowPath.Result(error_code=103)

    actions = [ActionServer(node, kind, name, execute_callback=invalid_path, callback_group=group)
               for kind, name in ((FollowPath, 'follow_path'), (NavigateToPose, 'navigate_to_pose'),
                                  (ComputePathToPose, 'compute_path_to_pose'))]
    tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = node.create_publisher(String, '/t1/sim_state', tl)
    amcl = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', tl)
    clock = node.create_publisher(Clock, '/clock', 10)
    scan = node.create_publisher(LaserScan, '/scan', 10)
    tf = TransformBroadcaster(node)
    tick = 0

    def publish():
        nonlocal tick
        tick += 1
        stamp = Clock().clock
        stamp.sec, stamp.nanosec = divmod(tick * 50_000_000, 1_000_000_000)
        clock.publish(Clock(clock=stamp))
        state.publish(String(data='{"state":"READY"}'))
        amcl.publish(PoseWithCovarianceStamped())
        msg = LaserScan()
        msg.header.stamp = stamp
        scan.publish(msg)
        transforms = []
        for parent, child in (('map', 'odom'), ('odom', 'base_footprint')):
            msg = TransformStamped(child_frame_id=child)
            msg.header.frame_id, msg.header.stamp = parent, stamp
            msg.transform.rotation.w = 1.
            transforms.append(msg)
        tf.sendTransform(transforms)

    timer = node.create_timer(.05, publish)
    yield
    timer.cancel()
    for action in actions:
        action.destroy()


CHILD = '''
import sys
from rclpy.client import Client
from t1_bringup.navigation_tools import starter_main
original = Client.destroy
def destroy(self):
    if self.srv_name == '/map_server/get_state':
        print('CLIENT_DESTROY_BARRIER', flush=True)
        if sys.stdin.readline().strip() == 'error':
            raise RuntimeError('INJECTED_UNEXPECTED_DESTRUCTOR_ERROR')
    return original(self)
Client.destroy = destroy
starter_main()
'''


@pytest.mark.parametrize('unexpected_error', [False, True])
def test_ready_starter_subprocess_single_sigint_during_client_cleanup(ready_endpoints, tmp_path, unexpected_error):
    stdout, stderr = tmp_path/'stdout.txt', tmp_path/'stderr.txt'
    with stdout.open('w') as out, stderr.open('w') as err:
        proc = subprocess.Popen(['/usr/bin/python3', '-u', '-c', CHILD, '--ros-args',
                                 '-p', 'use_sim_time:=true', '-p', 'ready_timeout_wall_s:=20.0'],
                                stdin=subprocess.PIPE, stdout=out, stderr=err, text=True,
                                start_new_session=True)
        try:
            deadline = time.monotonic() + 25.
            while 'CLIENT_DESTROY_BARRIER' not in stdout.read_text() and time.monotonic() < deadline:
                assert proc.poll() is None, (stdout.read_text(), stderr.read_text())
                time.sleep(.01)
            assert 'CLIENT_DESTROY_BARRIER' in stdout.read_text()
            report = next(json.loads(s) for s in stdout.read_text().splitlines() if s.startswith('{"starter"'))
            assert report['starter'] == 'navigation active' and report['exit'] == 0
            assert len(report['readiness']) == 11  # 8 states, real action ACK/result, 2 responses
            proc.send_signal(signal.SIGINT)  # exactly once, only after actual readiness
            proc.stdin.write('error\n' if unexpected_error else 'release\n')
            proc.stdin.flush()
            code = proc.wait(timeout=5.)
        finally:
            if proc.poll() is None:  # failure-only containment, never counted as a pass
                proc.kill()
                proc.wait(timeout=5.)
            proc.stdin.close()
    output = stdout.read_text() + stderr.read_text()
    assert 'KeyboardInterrupt' not in output
    if unexpected_error:
        assert code == 1
        assert 'Traceback (most recent call last)' in output
        assert 'INJECTED_UNEXPECTED_DESTRUCTOR_ERROR' in output
    else:
        assert code == 0
        assert 'Traceback' not in output
    # The directly launched starter owns no subprocesses. No process in its
    # new session/group may survive it (including an unexpected future child).
    with pytest.raises(ProcessLookupError):
        os.killpg(proc.pid, 0)


def test_cleanup_after_real_context_shutdown_is_idempotent(manager):
    with StarterRuntime() as runtime:
        runtime.initialize([])
        node = runtime.node
        node.create_timer(.05, lambda: None)
        node.create_client(GetState, '/absent')
        shutdowns = []
        runtime.context.on_shutdown(lambda: shutdowns.append('shutdown'))
        runtime.context.shutdown()
        assert not runtime.context.ok()
        runtime.cleanup()
        runtime.cleanup()
        assert shutdowns == ['shutdown']
        assert list(node.clients) == [] and list(node.timers) == []
        assert runtime.closed

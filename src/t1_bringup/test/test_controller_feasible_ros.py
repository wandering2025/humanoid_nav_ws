"""Real controller action: align before walking; near-goal walk stays feasible.

No simulator, truth, or test velocity publisher. TF/scan are synthetic inputs;
the installed controller publishes to an isolated topic in the DDS fixture.
"""
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from test_bounded_manager_ros import manager, response


@pytest.mark.parametrize('yaw', [0., 1.1594])
def test_controller_emits_executable_near_goal_intent(manager, tmp_path, yaw):
    node, start = manager
    ws = Path(__file__).resolve().parents[3]
    params = yaml.safe_load((ws/'src/t1_navigation/config/navigation.yaml').read_text())
    def wall_clock(tree):
        for key, value in tree.items():
            if key == 'use_sim_time':
                tree[key] = False
            elif isinstance(value, dict):
                wall_clock(value)
    wall_clock(params)
    config = tmp_path/'controller.yaml'
    config.write_text(yaml.safe_dump(params))
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'odom', 'base_footprint'
    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
    laser = TransformStamped()
    laser.header.frame_id, laser.child_frame_id = 'base_footprint', 'laser_stabilized'
    laser.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform([tf, laser])
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    def publish_scan():
        scan = LaserScan()
        scan.header.frame_id = 'laser_stabilized'
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2*math.pi/360
        scan.range_min, scan.range_max = .15, 12.
        scan.ranges = [float('inf')]*360
        scan_pub.publish(scan)
    node.create_timer(.1, publish_scan)
    commands = []
    node.create_subscription(Twist, '/test/controller_cmd', commands.append, 10)
    output = (tmp_path/'controller.log').open('w')
    proc = subprocess.Popen(['ros2', 'run', 'nav2_controller', 'controller_server',
        '--ros-args', '--params-file', str(config), '-r', 'cmd_vel:=/test/controller_cmd'],
        env=os.environ.copy(), stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    start.processes.append((proc, output))
    lifecycle = start('controller_server')
    assert response(lifecycle.call_async(ManageLifecycleNodes.Request(command=0))).success
    action = ActionClient(node, FollowPath, '/follow_path')
    assert action.wait_for_server(timeout_sec=5.)
    request = FollowPath.Goal()
    request.controller_id, request.goal_checker_id = 'FollowPath', 'general_goal_checker'
    request.path.header.frame_id = 'odom'
    for i in range(9):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = .05*i
        pose.pose.orientation.w = 1.
        request.path.poses.append(pose)
    handle = response(action.send_goal_async(request), 5.)
    assert handle.accepted
    end = time.monotonic()+5.
    while not commands and time.monotonic() < end:
        time.sleep(.02)
    assert commands
    command = commands[-1]
    if yaw:
        assert command.linear.x == 0.  # no sideways advance at the observed 66deg error
        assert -.60 <= command.angular.z <= -.05
    else:
        assert .315 <= command.linear.x <= .45  # executable W05 walk, even near goal
        assert abs(command.angular.z) <= .45
    assert response(handle.cancel_goal_async()).goals_canceling
    assert response(handle.get_result_async()).status == 5
    action.destroy()

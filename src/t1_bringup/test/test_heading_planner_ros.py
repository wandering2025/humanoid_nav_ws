"""Exercise the installed planner's returned path on the frozen static map.

No simulator or velocity publisher. A nominal start is supplied to the planning
action; assertions examine translation geometry, not the copied endpoint yaw.
"""
import math
import os
from pathlib import Path
import subprocess

import yaml
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster

from test_bounded_manager_ros import manager, response  # shared isolated DDS fixture


def test_detour_path_arrives_forward_along_goal_heading(manager, tmp_path):
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
    params['map_server'] = {'ros__parameters': {
        'use_sim_time': False, 'yaml_filename': str(ws/'src/t1_navigation/maps/static_world.yaml')}}
    path = tmp_path/'planner.yaml'
    path.write_text(yaml.safe_dump(params))
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'map', 'base_footprint'
    tf.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node)
    laser = TransformStamped()
    laser.header.frame_id, laser.child_frame_id = 'base_footprint', 'laser_stabilized'
    laser.transform.rotation.w = 1.
    laser.transform.translation.z = .8
    broadcaster.sendTransform([tf, laser])
    # Synthetic empty observations keep the real costmap's frozen 0.30 s
    # freshness requirement satisfied. The obstacle remains in the SLAM map.
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
    for package, executable in [('nav2_map_server', 'map_server'), ('nav2_planner', 'planner_server')]:
        output = (tmp_path/f'{executable}.log').open('w')
        proc = subprocess.Popen(['ros2', 'run', package, executable, '--ros-args',
                                 '--params-file', str(path)], env=os.environ.copy(),
                                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        start.processes.append((proc, output))
    lifecycle = start('map_server,planner_server')
    assert response(lifecycle.call_async(ManageLifecycleNodes.Request(command=0))).success
    action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    assert action.wait_for_server(timeout_sec=5.)
    request = ComputePathToPose.Goal()
    request.use_start = True
    request.planner_id = 'GridBased'
    request.start.header.frame_id = request.goal.header.frame_id = 'map'
    request.start.pose.orientation.w = request.goal.pose.orientation.w = 1.
    request.goal.pose.position.x = 9.
    handle = response(action.send_goal_async(request), 5.)
    assert handle.accepted
    result = response(handle.get_result_async(), 10.)
    assert result.status == 4 and result.result.error_code == 0
    poses = [p.pose for p in result.result.path.poses]
    assert len(poses) > 10
    assert max(abs(p.position.y) for p in poses) > 2.5  # actual island detour
    end = poses[-1].position
    # Inspect the incoming geometry at the frozen positional goal band. A
    # 0.5 m chord of a 1 m turn need not already be parallel to the endpoint.
    before = next(p.position for p in reversed(poses) if math.hypot(p.position.x-end.x, p.position.y-end.y) >= .2)
    assert abs(math.atan2(end.y-before.y, end.x-before.x)) < .20
    for a, b in zip(poses, poses[1:]):
        q = a.orientation
        yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        assert (b.position.x-a.position.x)*math.cos(yaw)+(b.position.y-a.position.y)*math.sin(yaw) >= -.001
    action.destroy()

"""Truth state export, independent of policy observations and ROS callbacks.

Free-joint qvel semantics are verified by W02/W06 mjOBJ_XBODY probes:
linear velocity is world-frame at trunk origin, angular velocity is body-frame.
base_footprint is a vertical projection, not a rigidly attached offset point.
"""
from dataclasses import dataclass
import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

ODOM = 'odom'
FOOTPRINT = 'base_footprint'
BASE = 'base_link'  # model_contract: trunk XBODY, world_joint at address zero


@dataclass(frozen=True)
class StateSnapshot:
    stamp_ns: int
    position: tuple
    quaternion_wxyz: tuple
    linear_world: tuple
    angular_body: tuple
    joint_names: tuple
    joint_position: tuple
    joint_velocity: tuple
    joint_effort: tuple


def capture(runner):
    """Copy ONLY on the MjData owner thread; no live data escapes."""
    runner.assert_owner()
    d = runner.mj_data
    return StateSnapshot(round(float(d.time) * 1_000_000_000),
                         tuple(d.qpos[:3]), tuple(d.qpos[3:7]),
                         tuple(d.qvel[:3]), tuple(d.qvel[3:6]),
                         tuple(runner.cfg.robot.joint_names),
                         tuple(d.qpos[runner.qadr]), tuple(d.qvel[runner.vadr]),
                         tuple(d.qfrc_actuator[runner.vadr]))


def normalize_wxyz(values):
    q = np.asarray(values, dtype=float)
    norm = float(np.linalg.norm(q))
    if q.shape != (4,) or not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError('invalid state quaternion')
    return q / norm


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v])


def rotation(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def stamp(target, ns):
    target.sec, target.nanosec = divmod(ns, 1_000_000_000)


def set_xyz(target, values):
    target.x, target.y, target.z = map(float, values)


def set_quaternion(target, wxyz):
    target.w, target.x, target.y, target.z = map(float, wxyz)


def nominal_covariance():
    # Ideal truth; nominal values, NOT a measured estimation uncertainty.
    return [1e-6 if i % 7 == 0 else 0.0 for i in range(36)]


def messages(s):
    """All messages derive from the same immutable post-step snapshot."""
    q = normalize_wxyz(s.quaternion_wxyz)
    w, x, y, z = q
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    c, sn = math.cos(yaw), math.sin(yaw)
    qyaw = np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)])
    residual = normalize_wxyz(multiply(qyaw * [1, -1, -1, -1], q))
    world_angular = rotation(q) @ s.angular_body
    vx, vy, _ = s.linear_world

    odom = Odometry()
    stamp(odom.header.stamp, s.stamp_ns)
    odom.header.frame_id, odom.child_frame_id = ODOM, FOOTPRINT
    set_xyz(odom.pose.pose.position, (s.position[0], s.position[1], 0.))
    set_quaternion(odom.pose.pose.orientation, qyaw)
    set_xyz(odom.twist.twist.linear, (c*vx+sn*vy, -sn*vx+c*vy, 0.))
    set_xyz(odom.twist.twist.angular, (0., 0., world_angular[2]))
    odom.pose.covariance = odom.twist.covariance = nominal_covariance()

    transforms = []
    for parent, child, pos, quat in (
            (ODOM, FOOTPRINT, (s.position[0], s.position[1], 0.), qyaw),
            (FOOTPRINT, BASE, (0., 0., s.position[2]), residual)):
        tf = TransformStamped()
        stamp(tf.header.stamp, s.stamp_ns)
        tf.header.frame_id, tf.child_frame_id = parent, child
        set_xyz(tf.transform.translation, pos)
        set_quaternion(tf.transform.rotation, quat)
        transforms.append(tf)

    joints = JointState()
    stamp(joints.header.stamp, s.stamp_ns)
    joints.name = list(s.joint_names)
    joints.position = list(map(float, s.joint_position))
    joints.velocity = list(map(float, s.joint_velocity))
    joints.effort = list(map(float, s.joint_effort))

    truth = Odometry()
    stamp(truth.header.stamp, s.stamp_ns)
    truth.header.frame_id, truth.child_frame_id = 'mj_world', 'base_link_truth'
    set_xyz(truth.pose.pose.position, s.position)
    set_quaternion(truth.pose.pose.orientation, q)
    set_xyz(truth.twist.twist.linear, rotation(q).T @ s.linear_world)
    set_xyz(truth.twist.twist.angular, s.angular_body)
    truth.pose.covariance = truth.twist.covariance = nominal_covariance()
    return odom, transforms, joints, truth


class StatePublisher:
    def __init__(self, node, joint_state_divisor=1):
        self.odom = node.create_publisher(Odometry, '/odom', 10)
        self.joints = node.create_publisher(JointState, '/joint_states', 10)
        self.truth = node.create_publisher(Odometry, '/t1/eval/ground_truth', 10)
        self.tf = TransformBroadcaster(node)
        self.divisor = joint_state_divisor
        self.episode, self.step = None, 0

    def publish(self, snapshot, episode):
        if episode != self.episode:
            self.episode, self.step = episode, 0
        odom, transforms, joints, truth = messages(snapshot)
        self.odom.publish(odom)
        self.tf.sendTransform(transforms)
        if self.step % self.divisor == 0:
            self.joints.publish(joints)
        self.truth.publish(truth)
        self.step += 1

import math
from types import SimpleNamespace
import numpy as np
import pytest
from t1_mujoco.state_bridge import StateSnapshot, capture, messages, multiply, normalize_wxyz, rotation
from t1_mujoco.simulation import parameter_defaults, validate_parameters


def snapshot(q, linear=(0., 1., .2), angular=(.2, -.3, .4)):
    return StateSnapshot(2_000_000_123, (1., 2., .7), tuple(q), linear, angular,
                         ('hip', 'knee'), (.1, .2), (.3, .4), (5., 6.))


def xyzw(q):
    return [q.x, q.y, q.z, q.w]


def wxyz(q):
    return [q.w, q.x, q.y, q.z]


@pytest.mark.parametrize('heading', [90., -90., 180., -135.])
def test_rotated_forward_velocity_and_ros_quaternion_order(heading):
    yaw = math.radians(heading)
    q = [math.cos(yaw/2)*3, 0, 0, math.sin(yaw/2)*3]  # normalize explicitly
    s = snapshot(q, (math.cos(yaw), math.sin(yaw), .2), (0,0,.4))
    odom, tf, joints, truth = messages(s)
    assert odom.twist.twist.linear.x == pytest.approx(1.)
    assert odom.twist.twist.linear.y == pytest.approx(0., abs=1e-12)
    assert odom.twist.twist.linear.z == 0
    assert odom.twist.twist.angular.z == pytest.approx(.4)
    assert xyzw(odom.pose.pose.orientation) == pytest.approx([0,0,math.sin(yaw/2),math.cos(yaw/2)])
    assert odom.pose.pose.position.z == 0 and truth.pose.pose.position.z == .7
    assert truth.twist.twist.linear.z == pytest.approx(.2)
    stamps = [m.header.stamp for m in [odom, *tf, joints, truth]]
    assert all((t.sec, t.nanosec) == (2, 123) for t in stamps)
    assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_footprint'
    assert [t.child_frame_id for t in tf] == ['base_footprint', 'base_link']


def test_tilted_tf_chain_reconstructs_trunk_and_body_twist():
    qyaw = [math.cos(.7), 0, 0, math.sin(.7)]
    qtilt = [math.cos(.2), math.sin(.2), 0, 0]
    q = multiply(qyaw, qtilt)
    s = snapshot(q)
    odom, tf, joints, truth = messages(s)
    reconstructed = multiply(wxyz(tf[0].transform.rotation), wxyz(tf[1].transform.rotation))
    assert rotation(reconstructed) == pytest.approx(rotation(q))
    assert tf[1].transform.translation.z == .7
    assert odom.twist.twist.angular.z == pytest.approx((rotation(q) @ s.angular_body)[2])
    assert [truth.twist.twist.linear.x, truth.twist.twist.linear.y, truth.twist.twist.linear.z] == pytest.approx(rotation(q).T @ s.linear_world)
    # Vertical projection has no spurious angular cross-product contribution.
    odom2 = messages(snapshot(q, angular=(0,0,0)))[0]
    assert odom.twist.twist.linear == odom2.twist.twist.linear
    assert joints.name == ['hip','knee']
    assert list(joints.position) == [.1,.2] and list(joints.velocity) == [.3,.4] and list(joints.effort) == [5.,6.]
    assert list(odom.pose.covariance)[::7] == [1e-6]*6


@pytest.mark.parametrize('q', [(0,0,0,0), (float('nan'),0,0,1), (float('inf'),0,0,1)])
def test_invalid_quaternion_is_rejected(q):
    with pytest.raises(ValueError, match='invalid state quaternion'):
        normalize_wxyz(q)


def test_snapshot_copies_live_arrays_and_requires_owner():
    d = SimpleNamespace(time=.02, qpos=np.arange(30, dtype=float),
                        qvel=np.arange(29, dtype=float), qfrc_actuator=np.arange(29, dtype=float)*2)
    runner = SimpleNamespace(mj_data=d, qadr=np.array([7,9]), vadr=np.array([6,8]),
                             cfg=SimpleNamespace(robot=SimpleNamespace(joint_names=['hip','knee'])),
                             assert_owner=lambda: None)
    s = capture(runner)
    d.qpos[:] = d.qvel[:] = d.qfrc_actuator[:] = -1
    assert s.joint_position == (7.,9.) and s.joint_velocity == (6.,8.) and s.joint_effort == (12.,16.)
    assert s.stamp_ns == 20_000_000
    def deny():
        raise RuntimeError('wrong owner')
    runner.assert_owner = deny
    with pytest.raises(RuntimeError, match='wrong owner'):
        capture(runner)


@pytest.mark.parametrize('divisor', [0,-1,1.5,True])
def test_schedule_rejects_invalid_divisor(divisor):
    p = parameter_defaults()
    p['joint_state_divisor'] = divisor
    with pytest.raises(ValueError, match='positive integer'):
        validate_parameters(p)

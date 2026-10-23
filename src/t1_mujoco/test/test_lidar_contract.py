"""ROS/frame/config contract; actual MuJoCo fixtures use explicit venv evidence script."""
from dataclasses import replace
import math
import numpy as np
import pytest
from t1_mujoco.lidar import scan_transform
from t1_mujoco.lidar_config import MOUNT
from t1_mujoco.state_bridge import StateSnapshot, multiply, rotation
from t1_mujoco.simulation import parameter_defaults, validate_parameters


@pytest.mark.parametrize('yaw', [0., math.pi/2, -2.])
def test_virtual_origin_matches_rigid_sensor_with_tilt(yaw):
    qy = np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)])
    q = multiply(qy, [math.cos(.15),math.sin(.15),0,0])
    s = StateSnapshot(123456789, (2.,3.,.7), tuple(q), (0.,)*3, (0.,)*3, (), (), (), ())
    tf = scan_transform(s)
    p = tf.transform.translation
    expected = np.array(s.position) + rotation(q) @ MOUNT
    actual = np.array([2.,3.,0.]) + rotation(qy) @ [p.x,p.y,p.z]
    assert actual == pytest.approx(expected, abs=1e-12)
    assert tf.header.frame_id == 'base_footprint' and tf.child_frame_id == 'laser_stabilized'
    assert tf.header.stamp.nanosec == s.stamp_ns
    assert tf.transform.rotation.w == 1 and tf.transform.rotation.x == 0
    # An incorrect static mount would fail while the torso rolls.
    assert p.y != 0


@pytest.mark.parametrize('name,value', [('lidar_dropout',-1.),('lidar_dropout',1.01),('lidar_dropout',float('nan')),('lidar_noise_std',-1.),('lidar_noise_std',float('inf')),('seed',-1)])
def test_invalid_sensor_config_fails_before_start(name,value):
    p=parameter_defaults();p[name]=value
    with pytest.raises(ValueError):validate_parameters(p)

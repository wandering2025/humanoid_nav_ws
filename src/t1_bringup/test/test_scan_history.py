"""Change detection must preserve real safety scans and occluded unknown space."""
import numpy as np
import pytest

from t1_bringup.scan_history import ScanHistory


def test_intruder_stays_transient_while_stopped_then_static_wall_returns():
    history = ScanHistory()
    angles = np.linspace(-.4, .4, 81)
    wall = 6/np.cos(angles)
    for tick in range(5):
        filtered, mask, _ = history.filter(wall, angles, (0,0,0), tick, .15, 12)
        assert not mask.any()
        np.testing.assert_equal(filtered, wall)
    intruder = wall.copy(); intruder[30:51] = 3/np.cos(angles[30:51])
    original = intruder.copy()
    for tick in range(5, 35):
        filtered, mask, _ = history.filter(intruder, angles, (0,0,0), tick, .15, 12)
        assert mask[30:51].all() and mask.sum() == 21
        assert np.isnan(filtered[mask]).all()
        np.testing.assert_equal(filtered[~mask], wall[~mask])
    np.testing.assert_equal(intruder, original)  # safety input is never mutated
    filtered, mask, _ = history.filter(wall, angles, (0,0,0), 35, .15, 12)
    assert not mask.any()


def test_never_invent_free_space_behind_newly_seen_object():
    history = ScanHistory(); angles = np.array([0.])
    for tick in range(5):
        _, mask, _ = history.filter([2.], angles, (0,0,0), tick, .15, 12)
        assert not mask.any()  # initially unknown static object remains
    assert (40,0) not in history.free  # occluded space is not cleared
    _, mask, _ = history.filter([4.], angles, (0,0,0), 5, .15, 12)
    assert not mask.any()


def test_scan_time_rotation_and_nonfinite_semantics():
    history = ScanHistory(); angles = np.array([0.,1.,2.])
    for tick in range(4):
        filtered, mask, _ = history.filter([5.,np.inf,np.nan], angles, (1.,2.,np.pi/2), tick, .15, 12)
        assert not mask.any() and np.isposinf(filtered[1]) and np.isnan(filtered[2])
    _, mask, _ = history.filter([2.,np.inf,np.nan], angles, (1.,2.,np.pi/2), 4, .15, 12)
    assert mask.tolist() == [True,False,False]
    with pytest.raises(ValueError, match='time'):
        history.filter([2.,np.inf,np.nan], angles, (1.,2.,np.pi/2), 4, .15, 12)


def test_static_wall_preserved_when_robot_moves_and_turns():
    history = ScanHistory(); angles = np.linspace(-.6,.6,121)
    for tick in range(20):
        x, heading = tick*.05, tick*.005
        wall = (6-x)/np.cos(angles+heading)
        _, mask, _ = history.filter(wall, angles, (x,0,heading), tick, .15, 12)
        assert not mask.any()

"""Fault injection at protocol boundaries, plus original failure pose regression."""
from concurrent.futures import Future
from types import SimpleNamespace
import math

import pytest

from t1_bringup.navigation_protocol import (
    ProtocolFailure, await_future, require_response, CommandEpoch, aligned_pass, StationaryWindow)


class Clock:
    now = 0.

    def spin(self):
        self.now += .02


def test_discovered_action_without_ack_fails_with_bounded_reason():
    clock, future = Clock(), Future()
    with pytest.raises(ProtocolFailure, match='FOLLOW_PATH_ACK_TIMEOUT'):
        await_future(future, clock.spin, .1, 'FOLLOW_PATH_ACK', lambda: clock.now)
    assert .1 <= clock.now < .13


def test_costmap_discovery_is_not_a_response(monkeypatch):
    clock = Clock()
    import t1_bringup.navigation_protocol as protocol
    monkeypatch.setattr(protocol, 'await_future', lambda f, spin, timeout, label:
                        await_future(f, spin, timeout, label, lambda: clock.now))
    removed = []
    future = Future()
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda r: future,
                             remove_pending_request=removed.append)
    with pytest.raises(ProtocolFailure, match='COSTMAP_RESPONSE_TIMEOUT'):
        require_response(client, object(), clock.spin, .1, 'COSTMAP_RESPONSE')
    assert removed == [future]


@pytest.mark.parametrize('pose', [(7.706993, -.281345, 1.655849), (7.650173, -.312482, 1.601123)])
def test_original_detour_handoff_is_rejected(pose):
    assert math.dist(pose[:2], (7.5, 0.)) < .35
    assert not aligned_pass(pose, (7.5, 0., 0.), .35)
    assert aligned_pass((7.5, .1, .1), (7.5, 0., 0.), .35)


def test_rebound_restarts_stationary_hold():
    stable = StationaryWindow()
    for i in range(25):
        assert not stable.update(i*.1, (0., 0., 0.))
    # Zero instantaneous angular velocity at 2.4 s is insufficient; rebound follows.
    for i in range(25, 32):
        assert not stable.update(i*.1, (0., 0., (i-24)*.05))
    for i in range(32, 56):
        assert not stable.update(i*.1, (0., 0., .35))
    assert stable.update(5.7, (0., 0., .35))


def moving_epoch():
    epoch = CommandEpoch()
    epoch.statuses('nav', [('n1', 10, 2)])
    epoch.statuses('follow', [('f1', 11, 2)])
    epoch.receive(.33, .1, 1.)
    assert epoch.command(1.01, .15) == (.33, .1)
    return epoch


def test_cancel_clears_cache_and_cannot_revive_old_goal():
    epoch = moving_epoch()
    assert epoch.statuses('nav', [('n1', 10, 3)])  # caller MUST latch W10 stop
    epoch.receive(.45, 0., 1.02)
    assert epoch.command(1.03, .15) is None
    assert not epoch.statuses('nav', [('n1', 10, 5)])  # one stop per goal
    assert epoch.command(1.04, .15) is None


def test_aborted_zero_error_cannot_authorize_commands_or_arrival():
    epoch = moving_epoch()
    assert epoch.statuses('nav', [('n1', 10, 6)])
    epoch.statuses('follow', [('late_follow', 11, 2)])
    epoch.receive(.45, 0., 2.)
    assert epoch.command(2., .15) is None


def test_preemption_waits_for_new_followpath_and_fresh_command():
    epoch = moving_epoch()
    assert not epoch.statuses('nav', [('n1', 10, 6), ('n2', 20, 2)])
    epoch.receive(.45, 0., 1.02)  # old controller still executing
    assert epoch.command(1.03, .15) is None
    epoch.statuses('follow', [('f1', 11, 2), ('f2', 21, 2)])
    assert epoch.command(1.03, .15) is None
    epoch.receive(.33, -.1, 1.04)
    assert epoch.command(1.05, .15) == (.33, -.1)


def test_late_follow_probe_cannot_move_without_navigation_goal():
    epoch = CommandEpoch()
    epoch.statuses('follow', [('probe', 11, 2)])
    epoch.receive(.33, .1, 1.)
    assert epoch.command(1., .15) is None


def test_readiness_missing_initialpose_and_tf_is_explicit(monkeypatch):
    from t1_bringup.navigation_readiness import Readiness
    ready = object.__new__(Readiness)
    ready.sensors = lambda: dict(initialpose_ack=False, map_odom=False)
    with pytest.raises(ProtocolFailure, match='initialpose_ack,map_odom'):
        ready.wait_sensors(0.)


def test_inactive_lifecycle_cannot_pass_readiness(monkeypatch):
    from t1_bringup.navigation_readiness import Readiness
    import t1_bringup.navigation_readiness as module
    ready = object.__new__(Readiness)
    ready.trace = []
    ready.spin = lambda: None
    ready.clients = {'controller_server': SimpleNamespace(wait_for_service=lambda **k: True)}
    monkeypatch.setattr(module, 'require_response', lambda *a:
                        SimpleNamespace(current_state=SimpleNamespace(id=2)))
    with pytest.raises(ProtocolFailure, match='NOT_ACTIVE:2'):
        ready.lifecycle(('controller_server',), float('inf'))
    assert not ready.trace

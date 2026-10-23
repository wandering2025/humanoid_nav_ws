"""Cover the ok()/publish() race without hiding errors in an active context."""
from types import SimpleNamespace
import pytest
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from t1_bringup.safety import shutdown_gate


def fixture(alive, fail=False, invalidate=False):
    state = dict(alive=alive, calls=[])
    def latch(reason):
        state['calls'].append(reason)
        if invalidate:
            state['alive'] = False
        if fail:
            raise _rclpy.RCLError('publisher context invalid')
    node = SimpleNamespace(context=SimpleNamespace(ok=lambda: state['alive']), latch=latch)
    return node, state


def test_live_context_emits_shutdown_stop():
    node, state = fixture(True)
    shutdown_gate(node)
    assert state['calls'] == ['SHUTDOWN']


def test_closed_context_skips_publish():
    node, state = fixture(False)
    shutdown_gate(node)
    assert state['calls'] == []


def test_signal_invalidates_between_check_and_publish():
    node, state = fixture(True, fail=True, invalidate=True)
    shutdown_gate(node)
    assert state['calls'] == ['SHUTDOWN'] and not state['alive']


def test_live_publish_errors_still_raise():
    node, state = fixture(True, fail=True)
    with pytest.raises(_rclpy.RCLError):
        shutdown_gate(node)

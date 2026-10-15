"""Shutdown ownership and error propagation, independent of navigation success."""
import signal
from unittest.mock import Mock

import pytest
from rclpy.executors import ExternalShutdownException

from t1_bringup.starter_runtime import StarterRuntime


def resources():
    runtime = StarterRuntime()
    calls = []
    timer = Mock()
    timer.cancel.side_effect = lambda: calls.append('cancel_timer')
    node = Mock(timers=[timer])
    node.destroy_node.side_effect = lambda: calls.append('destroy_node')
    executor = Mock()
    executor.remove_node.side_effect = lambda n: calls.append('detach')
    executor.shutdown.side_effect = lambda **kw: calls.append('stop_executor') or True
    context = Mock()
    context.try_shutdown.side_effect = lambda: calls.append('shutdown_context')
    runtime.node, runtime.executor, runtime.context = node, executor, context
    return runtime, calls, node, executor, context


def test_cleanup_is_ordered_and_idempotent():
    runtime, calls, node, executor, context = resources()
    runtime.cleanup()
    runtime.cleanup()
    assert calls == ['cancel_timer', 'detach', 'stop_executor', 'destroy_node', 'shutdown_context']
    node.destroy_node.assert_called_once()
    executor.shutdown.assert_called_once_with(timeout_sec=2.)
    context.try_shutdown.assert_called_once()


def test_sigint_during_client_destruction_does_not_unwind_cleanup():
    runtime, calls, node, _, _ = resources()
    previous = signal.getsignal(signal.SIGINT)

    def destroy():
        signal.raise_signal(signal.SIGINT)
        calls.append('all_clients_destroyed')

    node.destroy_node.side_effect = destroy
    with runtime:
        runtime.cleanup()
    assert runtime.interrupted
    assert calls[-2:] == ['all_clients_destroyed', 'shutdown_context']
    assert signal.getsignal(signal.SIGINT) == previous


def test_sigint_is_observed_at_spin_boundary():
    runtime, _, _, executor, _ = resources()
    with runtime:
        signal.raise_signal(signal.SIGINT)
        with pytest.raises(KeyboardInterrupt):
            runtime.spin()
    executor.spin_once.assert_not_called()


def test_closed_context_does_not_spin():
    runtime, _, _, executor, context = resources()
    context.ok.return_value = False
    with pytest.raises(ExternalShutdownException):
        runtime.spin()
    executor.spin_once.assert_not_called()
    runtime.cleanup()


def test_destructor_error_propagates_but_context_is_released():
    runtime, _, node, _, context = resources()
    node.destroy_node.side_effect = RuntimeError('unexpected destructor failure')
    previous = signal.getsignal(signal.SIGINT)
    with pytest.raises(RuntimeError, match='unexpected destructor failure'):
        with runtime:
            pass
    context.try_shutdown.assert_called_once()
    assert not runtime.closed
    assert signal.getsignal(signal.SIGINT) == previous


def test_executor_timeout_is_not_reported_as_clean_shutdown():
    runtime, _, node, executor, context = resources()
    executor.shutdown.side_effect = None
    executor.shutdown.return_value = False
    with pytest.raises(RuntimeError, match='STARTER_EXECUTOR_SHUTDOWN_TIMEOUT'):
        runtime.cleanup()
    node.destroy_node.assert_not_called()  # cannot destroy resources still in use
    context.try_shutdown.assert_called_once()


@pytest.mark.parametrize('exception,alive,expected', [
    (KeyboardInterrupt, True, 0),
    (ExternalShutdownException, False, 0),
    (ExternalShutdownException, True, ExternalShutdownException),
    (ValueError, True, ValueError),
])
def test_starter_only_classifies_expected_interrupts_as_normal(monkeypatch, exception, alive, expected):
    import t1_bringup.starter_runtime as module
    from t1_bringup.navigation_tools import starter
    runtime, _, _, _, context = resources()
    context.ok.return_value = alive
    runtime.initialize = Mock(side_effect=exception)
    monkeypatch.setattr(module, 'StarterRuntime', lambda: runtime)
    if expected == 0:
        assert starter([]) == 0
    else:
        with pytest.raises(expected):
            starter([])
    assert runtime.closed

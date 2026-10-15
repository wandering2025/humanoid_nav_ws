"""Advanced-only ordered lifecycle; the frozen SafetyGate behavior is reused."""
import signal
import threading
import rclpy
from rclpy.signals import SignalHandlerOptions
from .safety import SafetyGate, shutdown_gate


def main(args=None):
    # Default rclpy SIGINT can destroy context while take_message is converting a
    # message. Keep it alive until all callbacks stop; never suppress live errors.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop=threading.Event()
    previous={s:signal.signal(s,lambda *_:stop.set()) for s in (signal.SIGINT,signal.SIGTERM)}
    node=None
    try:
        node=SafetyGate()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.02)
    finally:
        try:
            if node is not None:
                shutdown_gate(node)
                node.destroy_node()
        finally:
            if rclpy.ok():rclpy.shutdown()
            for s,handler in previous.items():signal.signal(s,handler)
    return 0

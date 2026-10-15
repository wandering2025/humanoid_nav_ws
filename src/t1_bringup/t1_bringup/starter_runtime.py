"""Own the one-shot starter's ROS resources and cooperative SIGINT shutdown.

The signal handler only records intent: raising from it can interrupt a client
after Node.destroy_client has removed it from the node's ownership list. Polling
raises KeyboardInterrupt at a safe work boundary instead. Keep this handler
installed through cleanup, including context shutdown; do not retry partially
completed destructors or suppress their errors.
"""
import signal

from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


class StarterRuntime:
    def __init__(self):
        self.context = Context()
        self.node = None
        self.executor = None
        self.interrupted = False
        self.closed = False

    def __enter__(self):
        self.previous_sigint = signal.signal(signal.SIGINT, self.request_stop)
        return self

    def request_stop(self, signum, frame):
        self.interrupted = True

    def initialize(self, args=None):
        # A private context avoids rclpy's process-wide asynchronous shutdown
        # and the global executor's implicit context.on_shutdown callback.
        self.context.init(args=args)
        self.node = Node('t1_navigation_starter', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

    def spin(self):
        if self.interrupted:
            raise KeyboardInterrupt
        if not self.context.ok():
            raise ExternalShutdownException
        self.executor.spin_once(timeout_sec=.02)

    def cleanup(self):
        if self.closed:
            return
        try:
            # No more work is spun after entering cleanup. Stop timer scheduling
            # before detaching and shutting down the single-threaded executor.
            if self.node is not None:
                for timer in self.node.timers:
                    timer.cancel()
            if self.executor is not None:
                if self.node is not None:
                    self.executor.remove_node(self.node)
                if not self.executor.shutdown(timeout_sec=2.):
                    raise RuntimeError('STARTER_EXECUTOR_SHUTDOWN_TIMEOUT')
                self.executor = None
            if self.node is not None:
                self.node.destroy_node()  # owns subscriptions, clients, timers
                self.node = None
        finally:
            # Atomic/idempotent context API, not ok() followed by shutdown().
            # Also runs after an unexpected destructor error, which propagates.
            self.context.try_shutdown()
        self.closed = True

    def __exit__(self, exc_type, exc, traceback):
        try:
            self.cleanup()
        finally:
            signal.signal(signal.SIGINT, self.previous_sigint)

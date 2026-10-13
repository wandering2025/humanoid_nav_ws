"""Thread-safe inputs; callbacks never access MuJoCo or the policy."""
from dataclasses import dataclass, field
import math
import threading
import time
from queue import Queue


class InputBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = 'STARTING'
        self.sim_time = 0.0
        self.command = None
        self.ignored_components = 0

    def boundary(self, state, sim_time, *, clear=False):
        with self.lock:
            self.state, self.sim_time = state, sim_time
            if clear:
                self.command = None

    def receive(self, components, wall=None):
        with self.lock:
            if self.state != 'READY':
                return
            values = tuple(float(x) for x in components)
            if len(values) != 6:
                raise ValueError('Twist requires six components')
            if any(values[i] != 0 for i in (2, 3, 4)):
                self.ignored_components += 1
            self.command = (values, self.sim_time, time.monotonic() if wall is None else wall)

    def consume(self, sim_time, wall, timeout_sim, timeout_wall, debug, mapper=None):
        """Return (incoming, u_pol, status). mapper: calibrated CommandMap.guard."""
        with self.lock:
            state, command = self.state, self.command
        zero = (0.0, 0.0, 0.0)
        if state != 'READY':
            return zero, zero, 'BLOCKED_BY_STATE'
        if command is None:
            return zero, zero, 'IDLE' if debug or mapper is not None else 'UNCALIBRATED'
        values, received_sim, received_wall = command
        incoming = (values[0], values[1], values[5])
        if not all(math.isfinite(x) for x in values):
            return zero, zero, 'INVALID'
        if sim_time - received_sim > timeout_sim or wall - received_wall > timeout_wall:
            return incoming, zero, 'TIMEOUT'
        if not debug:
            if mapper is None:
                return incoming, zero, 'UNCALIBRATED'
            return incoming, mapper(incoming)[0], 'ACTIVE'
        # Experimental policy units only. These are upstream command bounds,
        # not a calibrated physical envelope. Never raise small commands.
        return incoming, tuple(max(-1.0, min(1.0, x)) for x in incoming), 'ACTIVE'


@dataclass
class Request:
    operation: str
    value: bool = False
    deadline: float = field(default_factory=lambda: time.monotonic() + 2.0)
    done: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)
    cancelled: bool = False
    result: tuple = (False, 'not processed')


class RequestQueue:
    def __init__(self):
        self.queue = Queue()

    def submit(self, operation, value=False):
        request = Request(operation, value)
        self.queue.put(request)
        if not request.done.wait(max(0, request.deadline - time.monotonic())):
            with request.lock:
                if not request.done.is_set():
                    request.cancelled = True
                    return False, 'request expired before a simulation boundary'
        return request.result

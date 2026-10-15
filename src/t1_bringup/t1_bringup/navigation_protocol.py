"""Finite W11R1 protocol primitives, independent of ROS for fault injection tests."""
import math
import time


class ProtocolFailure(RuntimeError):
    pass


def await_future(future, spin, timeout, label, clock=time.monotonic):
    end = clock() + timeout
    while not future.done() and clock() < end:
        spin()
    if not future.done():
        raise ProtocolFailure(label + '_TIMEOUT')
    result = future.result()
    if result is None:
        raise ProtocolFailure(label + '_NO_RESPONSE')
    return result


def require_response(client, request, spin, timeout, label):
    if not client.service_is_ready():
        raise ProtocolFailure(label + '_NOT_DISCOVERED')
    future = client.call_async(request)
    try:
        return await_future(future, spin, timeout, label)
    finally:
        if not future.done():
            client.remove_pending_request(future)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class StationaryWindow:
    def __init__(self, window=.5, hold=2.):
        self.window, self.hold = window, hold
        self.samples, self.since = [], None

    def update(self, now, pose):
        if self.samples and now <= self.samples[-1][0]:
            return False
        self.samples.append((now, pose))
        while len(self.samples) > 2 and self.samples[1][0] <= now - self.window:
            self.samples.pop(0)
        old_t, old = self.samples[0]
        dt = now - old_t
        if dt < self.window:
            return False
        still = math.dist(pose[:2], old[:2])/dt < .05 and abs(angle(pose[2]-old[2]))/dt < .05
        self.since = (now if self.since is None else self.since) if still else None
        return self.since is not None and now-self.since >= self.hold


class TerminalCorrections:
    """One shared correction budget, always verify the nominal pose after settling.

    A position miss needs the original position/heading, not a yaw-only residual
    adjustment. Rebound corrections integrate measured yaw error, capped at the
    existing 0.5 rad aim offset. Never infer success from result error_code=0.
    """
    def __init__(self, limit=3, xy=.20, yaw=.20, gain=1.):
        self.limit, self.xy, self.yaw, self.gain = limit, xy, yaw, gain
        self.used = 0

    def next(self, status, nominal, aim, settled):
        if status != 'SUCCEEDED':
            raise ProtocolFailure('ACTION_' + status)
        if settled is None:
            raise ProtocolFailure('TERMINAL_NOT_STATIONARY')
        exy = math.dist(settled[:2], nominal[:2])
        residual = angle(nominal[2]-settled[2])
        if exy <= self.xy and abs(residual) <= self.yaw:
            return None
        if self.used >= self.limit:
            raise ProtocolFailure('TERMINAL_CORRECTIONS_EXHAUSTED')
        self.used += 1
        if exy > self.xy:
            return nominal
        offset = angle(aim[2]-nominal[2]) + self.gain*residual
        return (nominal[0], nominal[1], nominal[2]+max(-.5, min(.5, offset)))


def overshot(here, goal):
    dx, dy = here[0]-goal[0], here[1]-goal[1]
    return dx*math.cos(goal[2])+dy*math.sin(goal[2]) > .30 and math.hypot(dx, dy) > .30


def forward_staging(goal, settled, distances, segment_free):
    """Choose an existing shorter staging segment after failed joint verification.

    Advancing by at least 0.30 m avoids another position-tolerance-only action.
    It is a Nav2 position goal, not a velocity trajectory or a reverse maneuver.
    The caller spends the same shared correction budget before using this choice.
    """
    for distance in distances:
        point = (goal[0]-distance*math.cos(goal[2]),
                 goal[1]-distance*math.sin(goal[2]), goal[2])
        forward = ((point[0]-settled[0])*math.cos(goal[2]) +
                   (point[1]-settled[1])*math.sin(goal[2]))
        if forward >= .30 and segment_free(distance):
            return distance, point
    return None


class CommandEpoch:
    """Twist has no UUID: admit it only after the current Nav2 + FollowPath ACK.

    New goal/status epochs invalidate cached raw commands. A delayed FollowPath
    from a failed/preempted NavigateToPose cannot authorize the new epoch.
    """
    def __init__(self):
        self.nav = self.follow = None
        self.raw = None
        self.cancelled = set()

    def statuses(self, kind, rows):
        # Rows: (UUID, acceptance timestamp in ns, action status).
        current = getattr(self, kind)
        newest = max(rows, key=lambda r: r[1], default=None)
        if newest is None or (current and newest[1] < current[1]):
            return False
        if newest != current:
            self.raw = None
        setattr(self, kind, newest)
        cancel = kind == 'nav' and newest[2] in (3, 5, 6) and newest[0] not in self.cancelled
        if cancel:
            self.cancelled.add(newest[0])
        return cancel

    def allowed(self):
        return bool(self.nav and self.follow and self.nav[2] in (1, 2) and
                    self.follow[2] in (1, 2) and self.follow[1] >= self.nav[1] and
                    self.nav[0] not in self.cancelled)

    def receive(self, vx, wz, now):
        if self.allowed():
            self.raw = (vx, wz, now)

    def command(self, now, timeout):
        if not self.allowed() or self.raw is None or now-self.raw[2] > timeout:
            return None
        return self.raw[:2]

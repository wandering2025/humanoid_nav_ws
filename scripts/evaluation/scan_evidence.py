"""Bounded evaluator-only deferral when scan-time evidence arrives out of order."""
from collections import deque
import math


class DeferredSamples:
    def __init__(self, wait_s=.5, capacity=25):
        if not math.isfinite(wait_s) or not 0 < wait_s <= 2 or capacity < 1:
            raise ValueError('invalid evidence wait/capacity')
        self.wait_s, self.capacity = wait_s, capacity
        self.pending = deque()
        self.stats = dict(received=0, sampled=0, expired=0, overflow=0,
                          max_pending=0, max_wait_s=0.,
                          missing_tf_attempts=0, missing_obstacle_attempts=0)

    def add(self, item, now):
        self.stats['received'] += 1
        if len(self.pending) == self.capacity:
            self.pending.popleft(); self.stats['overflow'] += 1
        self.pending.append((now, item))
        self.stats['max_pending'] = max(self.stats['max_pending'],len(self.pending))

    def drain(self, sample, now):
        waiting = deque()
        while self.pending:
            arrived, item = self.pending.popleft()
            age = now-arrived
            if age > self.wait_s:
                self.stats['expired'] += 1
            elif sample(item):
                self.stats['sampled'] += 1
                self.stats['max_wait_s'] = max(self.stats['max_wait_s'],age)
            else:
                waiting.append((arrived,item))
        self.pending = waiting

    def summary(self):
        return dict(self.stats,pending=len(self.pending),wait_limit_s=self.wait_s,
                    capacity=self.capacity)

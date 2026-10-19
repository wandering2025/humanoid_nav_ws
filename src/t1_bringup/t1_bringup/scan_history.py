"""Sensor-only change detection for the SLAM branch, never for collision safety.

Free evidence is ray traversals in odom, accumulated from an empty history.
An endpoint in repeatedly observed free space is transient. Connected scan
surfaces containing such endpoints are withheld, including while they stop.
No unobserved space behind a withheld return is marked free.
"""
import numpy as np


class ScanHistory:
    def __init__(self, resolution=.1, clearance=.35, confirmations=3):
        self.res, self.clearance, self.confirmations = resolution, clearance, confirmations
        self.free = {}
        self.last_stamp = None

    def filter(self, ranges, angles, pose, stamp, range_min, range_max):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('scan time must increase within one session')
        self.last_stamp = stamp
        raw = np.asarray(ranges, dtype=float)
        direction = np.column_stack((np.cos(angles+pose[2]), np.sin(angles+pose[2])))
        valid = np.isfinite(raw) & (raw >= range_min) & (raw < range_max)
        distance = np.where(valid, raw, 0.)
        points = np.asarray(pose[:2])+direction*distance[:, None]
        cells = np.floor(points/self.res).astype(np.int64)
        transient = np.array([bool(ok and self.free.get(tuple(cell), 0) >= self.confirmations)
                              for ok, cell in zip(valid, cells)])
        # Circular adjacent-beam connected components, including +/-pi seam.
        links = valid & np.roll(valid, -1) & (np.linalg.norm(points-np.roll(points, -1, axis=0), axis=1) < .3)
        stack = list(np.flatnonzero(transient))
        while stack:
            i = stack.pop()
            for j, linked in (((i+1) % len(raw), links[i]), ((i-1) % len(raw), links[(i-1) % len(raw)])):
                if linked and not transient[j]:
                    transient[j] = True; stack.append(j)
        # Integrate actual ray traversals only, ending before every real hit.
        # Count each cell once per scan, not once per intersecting beam.
        lengths = np.where(valid, raw-self.clearance, np.where(np.isposinf(raw), range_max, 0.))
        steps = np.arange(range_min, range_max, self.res*.7)
        xy = np.asarray(pose[:2])+direction[:, None, :]*steps[None, :, None]
        free_cells = np.floor(xy[steps[None, :] < lengths[:, None]]/self.res).astype(np.int64)
        for cell in np.unique(free_cells, axis=0):
            key = tuple(cell)
            self.free[key] = min(self.confirmations, self.free.get(key, 0)+1)
        if len(self.free) > 2_000_000:
            raise RuntimeError('scan history memory budget exhausted')
        filtered = raw.copy(); filtered[transient] = np.nan
        return filtered, transient, points

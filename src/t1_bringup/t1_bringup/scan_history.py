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
        self.surfaces = {}
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
        # Preserve previously stable surfaces despite angular projection/noise
        # near corners. Such surfaces also stop cluster propagation into walls.
        stable = np.array([bool(ok and any(self.surfaces.get((cell[0]+dx,cell[1]+dy),0)>=self.confirmations
                            for dx in range(-2,3) for dy in range(-2,3)))
                           for ok,cell in zip(valid,cells)])
        was_free = np.array([self.free.get(tuple(cell), 0) >= self.confirmations for cell in cells])
        transient = valid & ~stable & was_free
        # Circular adjacent-beam connected components, including +/-pi seam.
        links = valid & np.roll(valid, -1) & (np.linalg.norm(points-np.roll(points, -1, axis=0), axis=1) < .3)
        stack = list(np.flatnonzero(transient))
        while stack:
            i = stack.pop()
            for j, linked in (((i+1) % len(raw), links[i]), ((i-1) % len(raw), links[(i-1) % len(raw)])):
                if linked and not transient[j] and not stable[j]:
                    transient[j] = True; stack.append(j)
        # Never grow the static prior into previously free space: otherwise a
        # slowly moving body could drag its own 'static' neighborhood with it.
        for cell in np.unique(cells[valid & ~transient & ~was_free],axis=0):
            key=tuple(cell)
            self.surfaces[key]=min(self.confirmations,self.surfaces.get(key,0)+1)
        # Integrate actual ray traversals only, ending before every real hit.
        # Count each cell once per scan, not once per intersecting beam.
        # Only finite, measured surfaces certify long-term traversed free space.
        # Projected no-return bins can graze corners; preserve them in output
        # but do not promote them into permanent free-space evidence here.
        lengths = np.where(valid, raw-self.clearance, 0.)
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

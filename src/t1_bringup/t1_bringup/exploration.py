"""Map-only frontier candidates. No ROS, scene, route, truth or filesystem input."""
from collections import deque
import math

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label


def transform_point(point, pose):
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    return (x+c*point[0]-s*point[1], y+s*point[0]+c*point[1])


class ObservedMap:
    def __init__(self, data, resolution, origin, position, clearance=2.3, inflated=False):
        self.data = np.asarray(data, dtype=np.int16)
        if self.data.ndim != 2 or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid occupancy grid')
        self.res = resolution
        self.origin = origin  # SE(2), including nonzero OccupancyGrid origin yaw
        self.obstacle_threshold = 99 if inflated else 65
        free = (self.data >= 0) & (self.data < self.obstacle_threshold)
        known = self.data >= 0
        # Unknown outside the published rectangle is not free space.
        wall_gap = distance_transform_edt(np.pad(self.data < self.obstacle_threshold, 1))[1:-1, 1:-1]*resolution
        # Unknown is never a traversable center cell. Do not erode free space by
        # the entire footprint at unknown boundaries: the first SLAM grid has
        # sparse ray coverage. Nav2's inflated live costmap and the unchanged
        # LiDAR Collision Monitor enforce the physical body clearance as we move.
        self.safe = free & (wall_gap >= (.20 if inflated else clearance))
        self.frontier = free & binary_dilation(~known)
        self.frontier_gap = distance_transform_edt(~self.frontier)*resolution
        labels, _ = label(self.safe)  # four-connected; no diagonal corner cutting
        start = self.cell(position)
        self.reachable = np.zeros_like(self.safe)
        self.distance = np.full(self.data.shape, np.inf)
        if self.inside(start) and labels[start] != 0:
            self.reachable = labels == labels[start]
            queue = deque([start]); self.distance[start] = 0.
            while queue:
                j, i = queue.popleft()
                for y, x in ((j-1, i), (j+1, i), (j, i-1), (j, i+1)):
                    if (self.inside((y, x)) and self.reachable[y, x]
                            and not math.isfinite(self.distance[y, x])):
                        self.distance[y, x] = self.distance[j, i]+resolution
                        queue.append((y, x))

    def cell(self, point):
        x, y, a = self.origin
        dx, dy = point[0]-x, point[1]-y
        return (math.floor((-math.sin(a)*dx+math.cos(a)*dy)/self.res),
                math.floor((math.cos(a)*dx+math.sin(a)*dy)/self.res))

    def point(self, cell):
        j, i = cell
        return transform_point(((i+.5)*self.res, (j+.5)*self.res), self.origin)

    def inside(self, cell):
        return 0 <= cell[0] < self.data.shape[0] and 0 <= cell[1] < self.data.shape[1]

    def is_reachable(self, point):
        cell = self.cell(point)
        return self.inside(cell) and bool(self.reachable[cell])

    def path_known(self, points):
        # NavFn may use a nearby cell within its tolerance; also check endpoint
        # separately at the caller. Verify full segments, not only sparse poses.
        if not points:
            return False
        for a, b in zip(points, points[1:]):
            for t in np.linspace(0, 1, max(2, math.ceil(math.dist(a, b)/self.res)+1)):
                if not self.is_reachable((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))):
                    return False
        return self.is_reachable(points[0]) and self.is_reachable(points[-1])

    def information(self, point):
        # Estimated observable frontier angle, discounted by distance. Rays stop
        # at the first known obstacle or unknown cell; nothing behind a wall counts.
        gain = 0.
        for angle in np.linspace(-math.pi, math.pi, 72, endpoint=False):
            for d in np.arange(.3, 12.01, .3):
                cell = self.cell((point[0]+d*math.cos(angle), point[1]+d*math.sin(angle)))
                if not self.inside(cell) or self.data[cell] < 0:
                    gain += 1-d/12.3
                    break
                if self.data[cell] >= self.obstacle_threshold:
                    break
        return gain

    def candidates(self, position, goal, visited=(), failed=()):
        if not np.any(self.frontier):
            return []
        step = max(1, round(.5/self.res))
        cells = np.argwhere(self.reachable & (self.frontier_gap <= 4.0))
        proposals = []
        for j, i in cells:
            if j % step or i % step:
                continue
            point = self.point((j, i))
            if math.dist(point, position) < 1.5 or any(math.dist(point, p) < 1.5 for p in failed):
                continue
            repeat = sum(max(0., 2.5-math.dist(point, p)) for p in visited)
            score = -.65*self.distance[j, i]-.8*math.dist(point, goal)-8*repeat
            proposals.append(dict(point=list(point), score=float(score),
                                  travel_m=float(self.distance[j, i]), repeat=repeat))
        proposals.sort(key=lambda p: p['score'], reverse=True)
        spaced = []
        for p in proposals:
            if any(math.dist(p['point'], q['point']) < 1.5 for q in spaced):
                continue
            p['gain'] = self.information(p['point'])
            if p['gain'] < 1:
                continue
            p['score'] += .35*p['gain']
            spaced.append(p)
            if len(spaced) >= 24:
                break
        return sorted(spaced, key=lambda p: p['score'], reverse=True)

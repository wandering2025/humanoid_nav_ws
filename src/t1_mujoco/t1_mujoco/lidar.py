"""Owner-thread body-fixed raycast and gravity-aligned planar projection.

No scene metadata is used. Only actual MuJoCo intersections produce returns.
"""
from dataclasses import dataclass
import math
import time

import numpy as np

from .lidar_config import FRAME, RAW_FRAME, MOUNT, LidarConfig
from .state_bridge import normalize_wxyz, rotation, stamp, set_xyz


def pose(snapshot):
    q = normalize_wxyz(snapshot.quaternion_wxyz)
    w, x, y, z = q
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    c, s = math.cos(yaw), math.sin(yaw)
    level = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    origin = np.asarray(snapshot.position) + rotation(q) @ MOUNT
    return origin, level


def scan_transform(snapshot):
    from geometry_msgs.msg import TransformStamped
    origin, level = pose(snapshot)
    relative = level.T @ (origin - np.array([*snapshot.position[:2], 0.]))
    tf = TransformStamped()
    stamp(tf.header.stamp, snapshot.stamp_ns)
    tf.header.frame_id, tf.child_frame_id = 'base_footprint', FRAME
    set_xyz(tf.transform.translation, relative)
    tf.transform.rotation.w = 1.
    return tf


@dataclass
class ScanFrame:
    stamp_ns: int
    ranges: np.ndarray
    raw_points: np.ndarray
    geom_ids: np.ndarray
    world_hits: np.ndarray
    projected_hits: np.ndarray
    origin: np.ndarray
    level_rotation: np.ndarray
    compute_ms: float
    blind_bins: int
    known_fraction: float

    @property
    def healthy(self):
        return self.blind_bins == 0 and self.known_fraction >= .75


class RayLidar:
    def __init__(self, runner, config=LidarConfig()):
        import mujoco
        runner.assert_owner()
        self.runner, self.config = runner, config
        m = runner.mj_model
        self.body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'trunk')
        if self.body < 1:
            raise ValueError('lidar requires trunk body')
        # Descendants, not just one bodyexclude. World and independent mocap
        # bodies remain visible regardless of collision contype/conaffinity.
        bodies = {self.body}
        for i in range(self.body+1, m.nbody):
            if int(m.body_parentid[i]) in bodies:
                bodies.add(i)
        self.robot_geoms = np.isin(m.geom_bodyid, list(bodies))
        if np.any((m.geom_group == 5) & ~self.robot_geoms):
            raise ValueError('group 5 is reserved for all robot geoms; environment uses it')
        m.geom_group[self.robot_geoms] = 5
        self.mask = np.array([1, 1, 1, 1, 1, 0], dtype=np.uint8)
        self.increment = 2*math.pi/config.angles
        az = -math.pi + np.arange(config.angles)*self.increment
        el = np.deg2rad(config.elevations)
        a, e = np.meshgrid(az, el)
        self.directions = np.stack((np.cos(e)*np.cos(a), np.cos(e)*np.sin(a), np.sin(e)), axis=-1).reshape(-1, 3)
        self.dist = np.empty(len(self.directions))
        self.ids = np.empty(len(self.directions), dtype=np.int32)
        self.episode = None
        self.step = 0
        self.rng = np.random.default_rng(config.seed)

    def due(self, episode):
        self.runner.assert_owner()
        if episode != self.episode:
            self.episode, self.step = episode, 0
            self.rng = np.random.default_rng(self.config.seed)
        self.step += 1
        return self.step % self.config.divisor == 0

    def sample(self, snapshot):
        import mujoco
        self.runner.assert_owner()
        began = time.perf_counter()
        m, d, cfg = self.runner.mj_model, self.runner.mj_data, self.config
        if snapshot.stamp_ns != round(float(d.time)*1e9):
            raise ValueError('scan snapshot and physics timestamp differ')
        mujoco.mj_kinematics(m, d)  # mj_step leaves position caches one substep old
        origin, level = pose(snapshot)
        body_rotation = d.xmat[self.body].reshape(3, 3)
        actual_origin = d.xpos[self.body] + body_rotation @ MOUNT
        if not np.allclose(origin, actual_origin, atol=1e-10, rtol=0):
            raise ValueError('scan mount and state snapshot disagree')
        directions = np.ascontiguousarray(self.directions @ body_rotation.T)
        mujoco.mj_multiRay(m, d, origin, directions.ravel(), self.mask, True, -1,
                          self.ids, self.dist, None, len(self.dist), cfg.range_max)
        if (not np.isfinite(self.dist).all() or
                np.any((self.dist < 0) & (self.dist != -1)) or
                np.any((self.dist >= 0) & ((self.ids < 0) | (self.ids >= m.ngeom)))):
            raise ValueError('invalid raycast output')
        hit = (self.dist >= 0) & (self.dist <= cfg.range_max)
        if np.any(self.robot_geoms[self.ids[hit]]):
            raise ValueError('robot geometry leaked into lidar')
        raw = self.directions[hit] * self.dist[hit, None]
        world_hits = origin + directions[hit] * self.dist[hit, None]
        # Keep ALL raw in-range hits (including floor/overhead) for inspection.
        relative = directions @ level
        bins = np.floor((np.arctan2(relative[:, 1], relative[:, 0])+math.pi)/self.increment + .5).astype(int) % cfg.angles
        distance = self.dist.copy()
        dropped = self.rng.random(len(distance)) < cfg.dropout if cfg.dropout else np.zeros(len(distance), dtype=bool)
        if cfg.noise_std:
            distance[hit] += self.rng.normal(0, cfg.noise_std, int(hit.sum()))
        measured = origin + directions * np.maximum(distance, 0)[:, None]
        valid = hit & ~dropped & (measured[:, 2] >= cfg.height_min) & (measured[:, 2] <= cfg.height_max)
        xy_range = distance * np.linalg.norm(relative[:, :2], axis=1)
        valid &= (distance >= 0) & (distance <= cfg.range_max)
        nohit = ((self.dist == -1) | (self.dist > cfg.range_max)) & ~dropped
        # A filtered/dropout/unobserved bin stays unknown unless another actual
        # ray covers it. No interpolation or synthetic horizontal ray is used.
        ranges = np.full(cfg.angles, np.nan)
        ranges[bins[nohit]] = np.inf
        candidates = np.full(cfg.angles, np.inf)
        np.minimum.at(candidates, bins[valid], xy_range[valid])
        has_hit = np.zeros(cfg.angles, dtype=bool)
        has_hit[bins[valid]] = True
        ranges[has_hit] = candidates[has_hit]
        ranges[has_hit & (ranges < cfg.range_min)] = -np.inf
        return ScanFrame(snapshot.stamp_ns, ranges, raw, self.ids[hit].copy(), world_hits,
                         measured[valid].copy(), origin, level,
                         (time.perf_counter()-began)*1000,
                         int(np.isneginf(ranges).sum()), float((~np.isnan(ranges)).mean()))


class LidarPublisher:
    def __init__(self, node, config):
        from sensor_msgs.msg import LaserScan, PointCloud2
        self.config = config
        self.scan = node.create_publisher(LaserScan, '/scan', 5)
        self.points = node.create_publisher(PointCloud2, '/t1/lidar/points', 2) if config.publish_points else None

    def publish(self, frame):
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        cfg = self.config
        msg = LaserScan()
        stamp(msg.header.stamp, frame.stamp_ns)
        msg.header.frame_id = FRAME
        msg.angle_min = -math.pi
        msg.angle_increment = 2*math.pi/cfg.angles
        msg.angle_max = msg.angle_min + (cfg.angles-1)*msg.angle_increment
        msg.scan_time = .02*cfg.divisor
        msg.time_increment = 0.
        msg.range_min, msg.range_max = cfg.range_min, cfg.range_max
        msg.ranges = frame.ranges.astype(np.float32).tolist()
        self.scan.publish(msg)
        if self.points is not None:
            cloud = PointCloud2()
            stamp(cloud.header.stamp, frame.stamp_ns)
            cloud.header.frame_id = RAW_FRAME
            cloud.height, cloud.width = 1, len(frame.raw_points)
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.is_bigendian, cloud.is_dense = False, True
            cloud.point_step, cloud.row_step = 12, 12*cloud.width
            cloud.data = frame.raw_points.astype('<f4').tobytes()
            self.points.publish(cloud)

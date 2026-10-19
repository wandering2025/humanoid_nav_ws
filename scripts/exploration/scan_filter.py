"""W16 mapping-only scan filter. Inputs: timestamped raw LaserScan and odom TF."""
import argparse
import json
import math
from pathlib import Path
import signal
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from t1_bringup.scan_history import ScanHistory


class MappingScanFilter(Node):
    def __init__(self, output):
        super().__init__('t1_mapping_scan_filter')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.history, self.pending = ScanHistory(), deque(maxlen=10)
        self.log = output.open('w')
        self.publisher = self.create_publisher(LaserScan, '/scan/mapping', 5)
        self.create_subscription(LaserScan, '/scan', self.pending.append, 5)
        self.create_timer(.02, self.process)
        self.running = True

    def process(self):
        while self.pending:
            scan = self.pending[0]
            try:
                tf = self.buffer.lookup_transform('odom', scan.header.frame_id, Time.from_msg(scan.header.stamp))
            except Exception:
                # Wait for the exact scan-time TF, never silently use latest.
                return
            self.pending.popleft()
            t = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            q = tf.transform.rotation; p = tf.transform.translation
            pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
            begin = time.monotonic()
            filtered, mask, points = self.history.filter(scan.ranges, angles, pose, t, scan.range_min, scan.range_max)
            # NaN is ignored by installed Karto scan matching and rasterization.
            # Keep every other range and the original header/geometry unchanged.
            scan.ranges = filtered.astype(np.float32).tolist()
            self.publisher.publish(scan)
            self.log.write(json.dumps(dict(sim=t, pose=pose, masked_indices=np.flatnonzero(mask).tolist(),
                masked_points_odom=points[mask].tolist(), known_free_cells=len(self.history.free),
                processing_ms=(time.monotonic()-begin)*1000))+'\n')
            self.log.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MappingScanFilter(args.output)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(node, 'running', False))
    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

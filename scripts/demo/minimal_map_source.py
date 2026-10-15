"""CPU-only saved-map publisher for the single RViz lifecycle check; no physics."""
from cpu_guard import enter
enter()
import signal
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from grid_visuals import cloud

stop=False
def request(*_):
    global stop
    stop=True
signal.signal(signal.SIGINT,request);signal.signal(signal.SIGTERM,request)
root=Path(__file__).resolve().parents[2]
meta=yaml.safe_load((root/'src/t1_navigation/maps/dynamic_crossing_world.yaml').read_text())
with (root/'src/t1_navigation/maps'/meta['image']).open('rb') as f:
    assert f.readline().strip()==b'P5'
    header=[]
    while len(header)<3:header+=f.readline().split(b'#',1)[0].split()
    w,h,maximum=map(int,header);assert maximum==255
    pixels=np.frombuffer(f.read(),dtype=np.uint8).reshape(h,w)[::-1]
prob=1-pixels.astype(float)/255
grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=w;grid.info.height=h
grid.info.resolution=meta['resolution'];grid.info.origin.position.x=meta['origin'][0];grid.info.origin.position.y=meta['origin'][1];grid.info.origin.orientation.w=1.
grid.data=np.where(prob>meta['occupied_thresh'],100,np.where(prob<meta['free_thresh'],0,-1)).astype(np.int8).ravel().tolist()
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('w14_saved_map_only')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pubs=[(node.create_publisher(PointCloud2,'/t1/viz/'+kind+'_cells',qos),cloud(grid,kind)) for kind in ('map','global','local')]
last=0
try:
    while not stop:
        if time.monotonic()-last>=1:
            for pub,msg in pubs:pub.publish(msg)
            last=time.monotonic()
        rclpy.spin_once(node,timeout_sec=.1)
finally:
    node.destroy_node();rclpy.shutdown()

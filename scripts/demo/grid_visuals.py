"""Read-only occupancy-cell visualization; no map/nav/TF/command outputs."""
import math
import numpy as np


def cell_data(grid,kind='map'):
    values=np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
    mask=values>=0 if kind=='map' else values>0
    iy,ix=np.nonzero(mask);v=values[iy,ix].astype(np.uint32)
    x=(ix+.5)*grid.info.resolution;y=(iy+.5)*grid.info.resolution
    q=grid.info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    c,s=math.cos(yaw),math.sin(yaw)
    points=np.zeros((len(v),4),dtype='<f4')
    points[:,0]=grid.info.origin.position.x+c*x-s*y
    points[:,1]=grid.info.origin.position.y+s*x+c*y
    points[:,2]=-.03 if kind=='map' else (-.02 if kind=='global' else -.01)
    if kind=='map':
        grey=np.where(v>=65,35,np.where(v<=25,245,150)).astype(np.uint32)
        rgb=(grey<<16)|(grey<<8)|grey
    elif kind=='global':
        rgb=((80+v*175//100)<<16)|np.uint32(150) # blue to magenta; raw positive cells only
    else:
        rgb=np.uint32(255<<16)|((220-v*160//100)<<8) # orange to red
    points[:,3]=np.asarray(rgb,dtype='<u4').view('<f4')
    return points.tobytes(),len(v)


def cloud(grid,kind='map'):
    from sensor_msgs.msg import PointCloud2,PointField
    data,count=cell_data(grid,kind)
    msg=PointCloud2();msg.header=grid.header;msg.height=1;msg.width=count
    msg.fields=[PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','rgb'))]
    msg.is_bigendian=False;msg.point_step=16;msg.row_step=count*16;msg.is_dense=True;msg.data=data
    return msg


class GridVisuals:
    def __init__(self,node):
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.publishers=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for source,kind in (('/map','map'),('/global_costmap/costmap','global'),('/local_costmap/costmap','local')):
            pub=node.create_publisher(PointCloud2,'/t1/viz/'+kind+'_cells',qos)
            self.publishers.append(pub)
            node.create_subscription(OccupancyGrid,source,lambda grid,p=pub,k=kind:p.publish(cloud(grid,k)),qos)

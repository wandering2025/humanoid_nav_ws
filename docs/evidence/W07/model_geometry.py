"""Independent URDF FK and conservative MuJoCo geom projection for evidence."""
import itertools
import math
import xml.etree.ElementTree as ET
import mujoco
import numpy as np


def axis_rotation(axis, angle):
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
    x,y,z=axis;skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)


def origin(element):
    result=np.eye(4)
    if element is not None:
        result[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
        r,p,y=np.fromstring(element.get('rpy','0 0 0'),sep=' ')
        result[:3,:3]=axis_rotation([0,0,1],y)@axis_rotation([0,1,0],p)@axis_rotation([1,0,0],r)
    return result


class UrdfFK:
    def __init__(self,description):
        self.root=ET.fromstring(description)
        self.joints=[]
        for j in self.root.findall('joint'):
            self.joints.append((j.get('name'),j.get('type'),j.find('parent').get('link'),j.find('child').get('link'),origin(j.find('origin')),np.fromstring(j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0',sep=' ')))

    def poses(self, positions, base):
        result={'base_link':base};pending=list(self.joints)
        while pending:
            count=len(pending)
            for item in list(pending):
                name,kind,parent,child,t,axis=item
                if parent not in result:continue
                motion=np.eye(4)
                if kind in ('revolute','continuous'):
                    motion[:3,:3]=axis_rotation(axis,positions[name])
                elif kind!='fixed':raise ValueError(kind)
                result[child]=result[parent]@t@motion;pending.remove(item)
            if len(pending)==count:raise ValueError('URDF tree disconnected/cyclic')
        return result


def root_pose(data):
    rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,data.qpos[3:7])
    base=np.eye(4);base[:3,:3]=rotation.reshape(3,3);base[:3,3]=data.qpos[:3]
    return base


class GeometryBounds:
    def __init__(self, model):
        self.model=model;self.geoms=[]
        for i in range(model.ngeom):
            if model.geom_bodyid[i]==0:continue
            kind=int(model.geom_type[i]);size=model.geom_size[i]
            if kind==mujoco.mjtGeom.mjGEOM_MESH:
                mid=model.geom_dataid[i];start=model.mesh_vertadr[mid];n=model.mesh_vertnum[mid]
                vertices=model.mesh_vert[start:start+n]
                lo,hi=vertices.min(axis=0),vertices.max(axis=0)
            else:
                if kind==mujoco.mjtGeom.mjGEOM_SPHERE: half=np.full(3,size[0])
                elif kind==mujoco.mjtGeom.mjGEOM_CAPSULE: half=np.array([size[0],size[0],size[1]+size[0]])
                elif kind==mujoco.mjtGeom.mjGEOM_CYLINDER: half=np.array([size[0],size[0],size[1]])
                elif kind in (mujoco.mjtGeom.mjGEOM_BOX,mujoco.mjtGeom.mjGEOM_ELLIPSOID):half=size
                else:raise ValueError(f'unsupported body geom type {kind}')
                lo,hi=-half,half
            corners=np.array(list(itertools.product(*zip(lo,hi))))
            self.geoms.append((i,corners,bool(model.geom_contype[i] or model.geom_conaffinity[i])))

    def project(self,data):
        w,x,y,z=data.qpos[3:7];yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        c,s=math.cos(yaw),math.sin(yaw);rot=np.array([[c,-s],[s,c]])
        collision=[];all_points=[]
        for i,points,is_collision in self.geoms:
            world=points@data.geom_xmat[i].reshape(3,3).T+data.geom_xpos[i]
            plane=(world[:,:2]-data.qpos[:2])@rot
            all_points.extend(plane)
            if is_collision:collision.extend(plane)
        return np.asarray(collision),np.asarray(all_points)

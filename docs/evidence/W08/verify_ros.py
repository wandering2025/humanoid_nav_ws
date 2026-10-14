"""W08 actual launch, DDS/scan/TF, physical turn, pause/reset and RViz."""
from datetime import datetime,timezone
import json, math, os, signal, subprocess, sys, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan,PointCloud2
from std_srvs.srv import SetBool, Trigger
from t1_mujoco.state_bridge import rotation
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W08';RAW=OUT/'raw';RAW.mkdir(exist_ok=True)
sys.path.insert(0,str(WS/'docs/evidence/W06'))
import verify_integration as w06
w06.OUT=OUT;w06.RAW=RAW
old_source=w06.tf_source

def tf_source(message,static=False):
    if not static and {t.child_frame_id for t in message.transforms}=={'base_footprint','base_link','laser_stabilized'}:
        return 't1_sim (batch + graph ownership)'
    return old_source(message,static)
w06.tf_source=tf_source

class Probe(w06.Probe):
    def __init__(self,run_id):
        super().__init__(run_id)
        self.scans=[];self.best_effort_stamps=[];self.clouds=[];self.laser_tfs={}
        self.create_subscription(LaserScan,'/scan',self.scans.append,1000)
        self.create_subscription(LaserScan,'/scan',lambda m:self.best_effort_stamps.append(w06.ns(m.header.stamp)),QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PointCloud2,'/t1/lidar/points',self.clouds.append,1000)
    def tf(self,m,info):
        super().tf(m,info)
        for t in m.transforms:
            if t.child_frame_id=='laser_stabilized':self.laser_tfs[w06.ns(t.header.stamp)]=t
    def graph(self):
        super().graph()
        for topic in ('/scan','/t1/lidar/points'):
            self.topic_info[topic]=dict(publishers=[dict(node=i.node_name,type=i.topic_type,reliability=i.qos_profile.reliability.name,durability=i.qos_profile.durability.name,depth=i.qos_profile.depth) for i in self.get_publishers_info_by_topic(topic)],subscribers=[i.node_name for i in self.get_subscriptions_info_by_topic(topic)])
w06.Probe=Probe

def analyse_scan(run):
    n=run.node;odom={r[0]:r for r in n.rows['/odom']};truth={r[0]:r for r in n.rows['/t1/eval/ground_truth']};clocks={r[0] for r in n.rows['/clock']}
    errors=[];tf_errors=[];tf_rot_errors=[];stamp_good=[];stamps=[];xy=[];semantics=[];cloud_err=[]
    for m in n.scans:
        stamp=w06.ns(m.header.stamp);stamps.append(stamp)
        good=stamp in odom and stamp in n.laser_tfs and stamp in truth and stamp in clocks;stamp_good.append(good)
        if not good:continue
        r=odom[stamp];tf=n.laser_tfs[stamp];t=truth[stamp]
        c,s=math.cos(r[4]),math.sin(r[4]);level=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        origin=np.array([r[1],r[2],0])+level@w06.xyz(tf.transform.translation)
        expected=np.array(t[1:4])+rotation(t[4:8])@np.array([.1,0,.2])
        tf_errors.append(float(np.linalg.norm(origin-expected)))
        q=tf.transform.rotation;tf_rot_errors.append(max(abs(q.x),abs(q.y),abs(q.z),abs(q.w-1)))
        a=m.angle_min+np.arange(len(m.ranges))*m.angle_increment;d=np.array(m.ranges);valid=np.isfinite(d)
        points=origin+np.c_[d[valid]*np.cos(a[valid]),d[valid]*np.sin(a[valid]),np.zeros(valid.sum())]@level.T
        err=np.min(abs(points[:,:2,None]-np.array([[-2.,8.],[-5.,5.]])[None,:,:]).reshape(len(points),4),axis=1)
        errors.extend(err);xy.extend(points[:,:2])
        semantics.append(m.header.frame_id=='laser_stabilized' and len(d)==360 and m.time_increment==0 and abs(m.scan_time-.1)<1e-6 and abs(m.angle_max-(m.angle_min+359*m.angle_increment))<1e-6 and abs(m.range_min-.15)<1e-6 and m.range_max==12.)
    for cloud in n.clouds:
        stamp=w06.ns(cloud.header.stamp)
        if stamp not in truth:continue
        t=truth[stamp];R=rotation(t[4:8]);origin=np.array(t[1:4])+R@np.array([.1,0,.2])
        pts=np.frombuffer(cloud.data,dtype='<f4').reshape(-1,3)@R.T+origin
        wall=np.min(abs(pts[:,:2,None]-np.array([[-2.,8.],[-5.,5.]])[None,:,:]).reshape(len(pts),4),axis=1)
        # Raw includes floor; either wall or z=0 must explain every return.
        cloud_err.extend(np.minimum(wall,abs(pts[:,2])))
    dt=np.diff(stamps)*1e-9;graph=n.topic_info['/scan'];pub=graph['publishers']
    checks=dict(scan_type_qos_owner=len(pub)==1 and all(pub[0][k]==v for k,v in dict(node='t1_sim',type='sensor_msgs/msg/LaserScan',reliability='RELIABLE',durability='VOLATILE').items()),
        rviz_subscribes='rviz2' in graph['subscribers'],reliable_and_be_receive=len(n.best_effort_stamps)>.95*len(stamps),
        scan_semantics=all(semantics) and len(semantics)>100,scan_same_stamp_state=all(stamp_good),scan_monotonic=bool(np.all(dt>0)),scan_sim_10hz=abs(np.median(dt)-.1)<1e-8,
        laser_tf_origin=max(tf_errors)<1e-10,laser_tf_gravity_aligned=max(tf_rot_errors)<1e-12,wall_p95_under_8cm=np.percentile(errors,95)<=.08,wall_max_under_12cm=max(errors)<=.12,
        points_received=len(n.clouds)>.95*len(stamps),raw_points_frame=all(c.header.frame_id=='lidar_link' for c in n.clouds),raw_points_on_geometry=max(cloud_err)<2e-6,
        scan_tf_single_owner=n.parents['laser_stabilized']=='base_footprint' and len(n.authorities['laser_stabilized'])==1,
        rigid_mount_rsp=n.parents['lidar_link']=='trunk' and len(n.authorities['lidar_link'])==1)
    np.savez_compressed(RAW/'ros-wall-points.npz',xy=np.array(xy))
    return dict(checks={k:bool(v) for k,v in checks.items()},scans=len(stamps),best_effort_scans=len(n.best_effort_stamps),clouds=len(n.clouds),sim_hz=float(1/np.median(dt)),wall_p95_m=float(np.percentile(errors,95)),wall_max_m=float(max(errors)),tf_origin_max_m=max(tf_errors),raw_cloud_surface_error_m=float(max(cloud_err)),graph=graph)

assert os.environ.get('T1_LOCAL_DDS')=='1'
# Params override raw point publication, never alter user's startup docs.
params=RAW/'runtime-points.yaml';params.write_text((WS/'src/t1_mujoco/config/runtime.yaml').read_text().replace('lidar_publish_points: false','lidar_publish_points: true'))
rviz=RAW/'lidar.rviz';rviz.write_text((WS/'src/t1_description/config/scene.rviz').read_text().replace('Distance: 3','Distance: 16').replace('Pitch: 0.35','Pitch: 1.35').replace('Target Frame: base_footprint','Target Frame: odom'))
log=(RAW/'ros-dds.txt').open('w');server=subprocess.Popen([str(WS/'scripts/start_local_dds.sh')],stdout=log,stderr=subprocess.STDOUT)
started=datetime.now(timezone.utc).isoformat();run=None;result={};rclpy.init()
try:
    time.sleep(.4);assert server.poll() is None,'dedicated discovery server failed'
    # DDS remote endpoint history depth is reported as 0 on this installed RMW.
    # Verify the exact publisher factory's local rclpy profile independently.
    from t1_mujoco.lidar import LidarPublisher
    from t1_mujoco.lidar_config import LidarConfig
    qos_node=rclpy.create_node('w08_local_qos_check')
    qos=LidarPublisher(qos_node,LidarConfig()).scan.qos_profile
    local_qos=dict(depth=qos.depth,history=qos.history.name,reliability=qos.reliability.name,durability=qos.durability.name)
    qos_node.destroy_node()
    run=w06.Run('w08_lidar',rviz=True,extra_args=['scene:=calibration_world',f'params_file:={params}',f'rviz_config:={rviz}'])
    run.ready();run.duration(1.,(0.,0.));standing=run.screenshot('rviz-standing.png')
    run.duration(3.,(.4,0.));run.duration(4.,(0.,0.))
    run.until(lambda: math.degrees(w06.yaw(run.node.odom.pose.pose.orientation))>=110,(0.,.6),timeout=12)
    run.duration(4.,(0.,0.));heading=math.degrees(w06.yaw(run.node.odom.pose.pose.orientation))
    run.duration(4.,(.4,0.));moving=run.screenshot('rviz-moving.png');run.duration(4.,(0.,0.))
    run.service(run.node.pause_client,SetBool.Request(data=True));run.duration(.3,wall=True)
    count=len(run.node.scans);sim=run.node.sim
    run.duration(1.1,(.4,0.),wall=True)
    checks=dict(local_scan_qos=local_qos==dict(depth=5,history='KEEP_LAST',reliability='RELIABLE',durability='VOLATILE'),pause_no_new_scan=len(run.node.scans)==count and run.node.sim==sim,physical_turn_near_90=abs(heading-90)<20)
    run.service(run.node.pause_client,SetBool.Request(data=False));run.duration(.8)
    checks['resume_no_stale_command']=run.node.state['u_pol']==[0.,0.,0.]
    before=run.node.sim;laststamp=w06.ns(run.node.scans[-1].header.stamp);epoch=run.node.epoch
    run.service(run.node.reset_client,Trigger.Request());run.until(lambda:run.node.epoch==epoch+1 and run.node.state['state']=='READY');run.duration(2.,(0.,0.))
    checks['reset_monotonic']=run.node.sim>before and w06.ns(run.node.scans[-1].header.stamp)>laststamp
    run.node.graph();detail=analyse_scan(run);checks.update(detail['checks'])
    result=dict(code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),started_utc=started,checks=checks,scan=detail,heading_after_turn_deg=heading,local_publisher_qos=local_qos,screenshots=[standing,moving],launch_pid=run.proc.pid)
finally:
    if run is not None:
        actual=run.close();result['run']=actual
        if result.get('checks'):
            result['checks'].update(actual['checks'])
            result['checks'].update(launch_exit_0=actual['exit_code']==0 and actual['manifest']['exit_code']==0,
                venv_interpreter=actual['manifest']['executable']==str(Path(os.environ['T1_VENV'])/'bin/python'),sensor_mode=actual['manifest']['sensor_mode']=='compensated')
    rclpy.shutdown()
    if server.poll() is None:server.send_signal(signal.SIGINT)
    server.wait(timeout=5);log.close()
    result.update(ended_utc=datetime.now(timezone.utc).isoformat(),server_pid=server.pid,server_exit=server.returncode,verdict='PASS' if result.get('checks') and all(result['checks'].values()) else 'FAIL')
    (OUT/'ros-rviz.json').write_text(json.dumps(result,indent=2)+'\n')
print(result.get('checks'),result['verdict']);assert result['verdict']=='PASS'

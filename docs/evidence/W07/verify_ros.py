"""W07 actual launch, named scene, RSP FK, ownership and RViz evidence.

Reuses W06's read-only ROS probe/lifecycle; redirects outputs to W07.
"""
from datetime import datetime,timezone
import json, os, signal, subprocess, sys, time
from pathlib import Path
import numpy as np
import rclpy
from tf2_msgs.msg import TFMessage
from t1_bringup.description import robot_description
from t1_mujoco.state_bridge import rotation
from model_geometry import UrdfFK
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W07';RAW=OUT/'raw';RAW.mkdir(exist_ok=True)
sys.path.insert(0,str(WS/'docs/evidence/W06'))
import verify_integration as w06
w06.OUT=OUT;w06.RAW=RAW
fk=UrdfFK(robot_description(Path(os.environ['T1_UPSTREAM']).parent/'booster_assets'))
assert os.environ.get('T1_LOCAL_DDS')=='1'
log=(RAW/'ros-dds.txt').open('w');server=subprocess.Popen([str(WS/'scripts/start_local_dds.sh')],stdout=log,stderr=subprocess.STDOUT)
started=datetime.now(timezone.utc).isoformat();run=None;result={};rclpy.init()
try:
    time.sleep(.4);assert server.poll() is None,'dedicated discovery server failed'
    run=w06.Run('w07_static',rviz=True,extra_args=['scene:=static_world'])
    received=[]
    run.node.create_subscription(TFMessage,'/tf',lambda m: received.extend(m.transforms),1000)
    run.ready();run.duration(1.,(0.,0.));standing=run.screenshot('rviz-standing.png')
    run.duration(2.,(.4,0.));run.duration(2.,(.4,.3));moving=run.screenshot('rviz-moving.png')
    run.duration(4.,(0.,0.));run.duration(2.,(0.,.6));run.duration(4.,(0.,0.))
    joints={row[0]:dict(zip(row[1],row[2])) for row in run.node.rows['/joint_states']}
    pose_cache={};errors=[];children=set()
    for tf in received:
        stamp=w06.ns(tf.header.stamp)
        if stamp not in joints or tf.child_frame_id in ('base_link','base_footprint'):continue
        if stamp not in pose_cache:pose_cache[stamp]=fk.poses(joints[stamp],np.eye(4))
        poses=pose_cache[stamp]
        expected=np.linalg.inv(poses[tf.header.frame_id])@poses[tf.child_frame_id]
        q=tf.transform.rotation;p=tf.transform.translation
        actual=rotation([q.w,q.x,q.y,q.z])
        errors.append([float(np.linalg.norm(expected[:3,3]-[p.x,p.y,p.z])),float(np.max(abs(expected[:3,:3]-actual)))])
        children.add(tf.child_frame_id)
    max_errors=np.max(errors,axis=0).tolist()
    expected_children={j[3] for j in fk.joints if j[1]!='fixed'}
    result=dict(started_utc=started,code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),rsp_fk=dict(samples=len(errors),children=sorted(children),max_translation_m=max_errors[0],max_rotation_element=max_errors[1]),launch_pid=run.proc.pid,screenshots=[standing,moving],checks=dict(rsp_fk_matches=len(errors)>1000 and max(max_errors)<1e-10 and children==expected_children))
    run.node.graph()
    # Save one observed JointState/truth snapshot for independent MuJoCo render.
    truth={row[0]:row for row in run.node.rows['/t1/eval/ground_truth']}
    common=max(set(joints)&set(truth))
    (RAW/'visual-snapshot.json').write_text(json.dumps(dict(joints=joints[common],truth=truth[common],stamp_ns=common))+'\n')
finally:
    if run is not None:
        actual=run.close();result['run']=actual
        if result.get('checks'):
            result['checks'].update(actual['checks'])
            p=actual['manifest'].get('scene_provenance',{})
            result['checks'].update(launch_exit_0=actual['exit_code']==0 and actual['manifest']['exit_code']==0,
                                   scene_loaded=p.get('name')=='static_world',
                                   venv_interpreter=actual['manifest']['executable']==str(Path(os.environ['T1_VENV'])/'bin/python'))
    rclpy.shutdown()
    if server.poll() is None:server.send_signal(signal.SIGINT)
    server.wait(timeout=5);log.close()
    result.update(ended_utc=datetime.now(timezone.utc).isoformat(),server_pid=server.pid,server_exit=server.returncode,verdict='PASS' if result.get('checks') and all(result['checks'].values()) else 'FAIL')
    (OUT/'ros-rviz.json').write_text(json.dumps(result,indent=2)+'\n')
print(result.get('checks'),result['verdict']);assert result['verdict']=='PASS'

"""Check live use_sim_time parameters, late-join QoS and 25 Hz joint divisor."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from verify_integration import Run, WS, OUT, RAW, ns

assert os.environ.get('T1_LOCAL_DDS')=='1'
params=WS/'run_data/w06-divisor2.yaml'
params.write_text('t1_sim:\n  ros__parameters:\n    joint_state_divisor: 2\n')
log=(RAW/'consumers-dds.txt').open('w')
server=subprocess.Popen([str(WS/'scripts/start_local_dds.sh')],stdout=log,stderr=subprocess.STDOUT)
started=datetime.now(timezone.utc).isoformat()
rclpy.init();run=None;late=None;result={}
try:
    time.sleep(.4);assert server.poll() is None
    run=Run('consumers',rviz=True,extra_args=[f'params_file:={params}'])
    run.ready()
    actual={}
    for name in ['t1_sim','robot_state_publisher','rviz2']:
        client=run.node.create_client(GetParameters, f'/{name}/get_parameters')
        assert client.wait_for_service(timeout_sec=3),name
        future=client.call_async(GetParameters.Request(names=['use_sim_time']))
        run.until(future.done,timeout=3)
        actual[name]=future.result().values[0].bool_value
        run.node.destroy_client(client)
    actual['w06_probe']=run.node.get_parameter('use_sim_time').value
    # Create subscriber after static TF/robot_description were already published.
    late=Node('w06_late_join',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    static=[];description=[]
    qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    late.create_subscription(TFMessage,'/tf_static',lambda m: static.extend(m.transforms),qos)
    late.create_subscription(String,'/robot_description',lambda m: description.append(m.data),qos)
    deadline=time.monotonic()+4
    while time.monotonic()<deadline and not (static and description):
        run.spin((0.,0.));rclpy.spin_once(late,timeout_sec=.005)
    run.duration(3.,(0.,0.))
    checks=dict(use_sim_time_correct=actual==dict(t1_sim=False,robot_state_publisher=True,rviz2=True,w06_probe=True),
                static_tf_late_join={t.child_frame_id for t in static}=={'trunk','head_realsense_rgb_link'},
                description_late_join=bool(description) and 'base_link_to_trunk' in description[0])
    static_stamps=[ns(t.header.stamp) for t in static]
    result=dict(started_utc=started,use_sim_time=actual,static_tf_stamps_ns=static_stamps,
                static_tf_children=[t.child_frame_id for t in static],checks=checks)
finally:
    if late is not None:late.destroy_node()
    if run is not None:
        run_result=run.close()
        if result:
            result['run']=run_result
            result['checks'].update(joints_25hz=abs(run_result['topics']['/joint_states']['median_sim_hz']-25)<.1,
                                    rsp_25hz=abs(run_result['topics']['/tf:left_ankle_roll_link']['median_sim_hz']-25)<.1,
                                    odom_clock_50hz=abs(run_result['topics']['/odom']['median_sim_hz']-50)<.1 and abs(run_result['topics']['/clock']['median_sim_hz']-50)<.1,
                                    tf_no_persistent_errors=run_result['tf_persistent_failures']==0,
                                    launch_exit_0=run_result['exit_code']==0 and run_result['manifest']['exit_code']==0)
            # Generic Run assumes default divisor=1; this deliberate variant uses 2.
            run_result['checks'].pop('rates_50hz',None)
            run_result['checks']['configured_rates_correct']=result['checks']['odom_clock_50hz'] and result['checks']['joints_25hz'] and result['checks']['rsp_25hz']
            run_result['rate_contract_note']='odom/clock/base TF/truth 50 Hz; joints/RSP joint TF configured 25 Hz'
    rclpy.shutdown()
    if server.poll() is None:server.send_signal(signal.SIGINT)
    server.wait(timeout=5);log.close()
    result.update(ended_utc=datetime.now(timezone.utc).isoformat(),server_exit=server.returncode,
                  verdict='PASS' if result.get('checks') and all(result['checks'].values()) else 'FAIL')
    (OUT/'consumer-qos.json').write_text(json.dumps(result,indent=2)+'\n')
print(result['checks'],result['verdict'])
assert result['verdict']=='PASS'

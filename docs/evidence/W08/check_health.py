"""Real ROS sensor coverage gate and exception path; owned processes only."""
from datetime import datetime,timezone
import csv,json,os,signal,subprocess,time,uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W08';RAW=OUT/'raw'
started=datetime.now(timezone.utc).isoformat();results=[]
assert os.environ.get('T1_LOCAL_DDS')=='1'
log=(RAW/'health-dds.txt').open('w');server=subprocess.Popen([str(WS/'scripts/start_local_dds.sh')],stdout=log,stderr=subprocess.STDOUT)
rclpy.init()
try:
    time.sleep(.4);assert server.poll() is None
    for fault in ('dropout','exception'):
        rid=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_dev_w08_'+fault+'_s0_'+uuid.uuid4().hex[:6]
        node=Node('w08_health_probe');states=[];scans=[];diags=[]
        node.create_subscription(String,'/t1/sim_state',lambda m:states.append(json.loads(m.data)),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(LaserScan,'/scan',scans.append,1000)
        node.create_subscription(DiagnosticArray,'/diagnostics',diags.append,100)
        pub=node.create_publisher(Twist,'/cmd_vel',1)
        cmd=[str(Path(os.environ['T1_VENV'])/'bin/python')]
        cmd+=['-m','t1_mujoco.simulation'] if fault=='dropout' else [str(OUT/'inject_sensor_failure.py')]
        cmd+=['--ros-args','-p',f'run_id:={rid}','-p','scene:=calibration_world','-p','max_sim_time:=6.0','-p',f'command_map_file:={WS}/src/t1_mujoco/config/command_map.yaml']
        if fault=='dropout':cmd+=['-p','lidar_dropout:=1.0']
        logfile=(RAW/(rid+'.txt')).open('w');proc=subprocess.Popen(cmd,stdout=logfile,stderr=subprocess.STDOUT);began=time.monotonic()
        try:
            while proc.poll() is None:
                assert time.monotonic()-began<25,'health run timed out'
                rclpy.spin_once(node,timeout_sec=.01)
                m=Twist();m.linear.x=.4;pub.publish(m)
            proc.wait(timeout=5)
            for _ in range(30):rclpy.spin_once(node,timeout_sec=.01)
        finally:
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
            proc.wait(timeout=8);logfile.close();node.destroy_node()
        manifest=json.loads((WS/'run_data'/rid/'manifest.json').read_text())
        trace=list(csv.DictReader((WS/'run_data'/rid/'sim_trace.csv').open()))
        events=[json.loads(line) for line in (WS/'run_data'/rid/'events.jsonl').read_text().splitlines()]
        relevant=[s for s in states if s.get('run_id')==rid]
        checks=dict(venv=manifest['executable']==str(Path(os.environ['T1_VENV'])/'bin/python'),sensor_mode=manifest['sensor_mode']=='compensated',diagnostic_error_received=any(s.level==2 for m in diags for s in m.status),states_received=bool(relevant))
        if fault=='dropout':
            checks.update(exit_0=proc.returncode==0,unknown_scans=len(scans)>30 and all(np.isnan(m.ranges).all() for m in scans),
                zero_policy_throughout=all(float(r['u_pol_'+a])==0 for r in trace for a in ('x','y','wz')),
                ready_but_blocked=any(s['state']=='READY' and not s['sensor_healthy'] and s['cmd_status']=='BLOCKED_BY_STATE' for s in relevant),
                continued_standing=float(trace[-1]['sim_time'])>=5.99 and float(trace[-1]['z_w'])>.6)
        else:
            faults=[s for s in relevant if s['state']=='FAULT' and s['fault']=='sensor_exception']
            checks.update(exit_2=proc.returncode==2 and manifest['exit_code']==2,
                fault_and_zero_received=bool(faults) and faults[-1]['u_pol']==[0.,0.,0.],
                no_failed_frame=len(scans)>20 and max(m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in scans)<4.,
                stops_at_failure=abs(manifest['sim_time_end']-4)<1e-8,
                fault_recorded=any(s.get('transition')=='FAULT' and s.get('fault')=='sensor_exception' for s in events),
                moving_before_fault=any(float(r['u_pol_x'])>0 for r in trace))
        result=dict(fault=fault,run_id=rid,pid=proc.pid,exit_code=proc.returncode,command=cmd,wall_s=time.monotonic()-began,scans=len(scans),checks={k:bool(v) for k,v in checks.items()},manifest=manifest)
        results.append(result);print(fault,result['checks'],flush=True)
finally:
    rclpy.shutdown()
    if server.poll() is None:server.send_signal(signal.SIGINT)
    server.wait(timeout=5);log.close()
    result=dict(code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),server_pid=server.pid,server_exit=server.returncode,runs=results,verdict='PASS' if len(results)==2 and all(all(r['checks'].values()) for r in results) else 'FAIL')
    (OUT/'health.json').write_text(json.dumps(result,indent=2)+'\n')
assert result['verdict']=='PASS'

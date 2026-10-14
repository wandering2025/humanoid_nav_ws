"""Safety-enabled mapping/localization graph, actual RMW TF authority regression."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger

WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W10';RAW=OUT/'raw'
results=[]
dlog=(RAW/'dds-modes.txt').open('w')
dds=subprocess.Popen(['bash',str(WS/'scripts/start_local_dds.sh')],stdout=dlog,stderr=subprocess.STDOUT)
time.sleep(.5);rclpy.init()
try:
    for mode in ('mapping','localization'):
        run_id='w10_'+mode+'_'+uuid.uuid4().hex[:8]
        node=Node('w10_mode_probe');state={};maps=[]
        pub=node.create_publisher(Twist,'/t1/cmd_teleop',1)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String,'/t1/safety/state',lambda m:state.update(json.loads(m.data)),qos)
        node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append((m.info.width,m.info.height)),qos)
        log=(RAW/(run_id+'.txt')).open('w')
        proc=subprocess.Popen(['ros2','launch','t1_bringup',mode+'.launch.py','run_id:='+run_id,'max_sim_time:=25.0'],stdout=log,stderr=subprocess.STDOUT)
        apath=RAW/(run_id+'-authority.jsonl');alog=apath.open('w')
        authority=subprocess.Popen([str(WS/'docs/evidence/W09/raw/authority-build/tf_authority')],stdout=alog,stderr=subprocess.DEVNULL)
        init=None;client=None
        try:
            if mode=='localization':
                # W09 trial-1 approximate map pose; the tool sets use_sim_time itself.
                init=subprocess.Popen(['ros2','run','t1_bringup','initial_pose','0.2','-0.15','0.15'],stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+35;future=None
            while proc.poll() is None:
                assert time.monotonic()<deadline
                rclpy.spin_once(node,timeout_sec=.01);pub.publish(Twist())
                if state and not state['health_reason'] and client is None:
                    client=node.create_client(Trigger,'/t1/safety/arm')
                if client is not None and future is None and client.service_is_ready():
                    future=client.call_async(Trigger.Request())
                if state.get('state')=='ARMED' and maps and time.monotonic()>deadline-22 and (init is None or init.poll() is not None):
                    break
            graph={t:{'publishers':[i.node_name for i in node.get_publishers_info_by_topic(t)],
                      'subscribers':[i.node_name for i in node.get_subscriptions_info_by_topic(t)]}
                for t in ('/cmd_vel','/cmd_vel_nav','/cmd_vel_smoothed','/clock','/t1/eval/ground_truth','/t1/debug/policy_cmd','/map')}
            authority.send_signal(signal.SIGINT);authority.wait(timeout=5);alog.close()
            auth=[json.loads(line) for line in apath.read_text().splitlines()]
            resolved=[a for a in auth if a['node']!='unresolved']
            owners={}
            for a in resolved:owners.setdefault(a['child'],set()).add(a['node'])
            checks=dict(armed=state.get('state')=='ARMED',map_received=bool(maps),
                unique_map_tf=owners.get('odom')=={'slam_toolbox' if mode=='mapping' else 'amcl'},
                unique_tf=all(len(v)==1 for v in owners.values()),
                sim_tf=owners.get('base_footprint')=={'t1_sim'},
                final_owner=graph['/cmd_vel']['publishers']==['collision_monitor'],
                clock_owner=graph['/clock']['publishers']==['t1_sim'],truth_isolated=not graph['/t1/eval/ground_truth']['subscribers'],
                debug_disabled=not graph['/t1/debug/policy_cmd']['subscribers'])
            if init is not None:checks['initial_pose_ok']=init.returncode==0
        finally:
            if proc.poll() is None:proc.send_signal(signal.SIGINT)
            proc.wait(timeout=15)
            if authority.poll() is None:authority.send_signal(signal.SIGINT);authority.wait(timeout=5)
            if init is not None:
                if init.poll() is None:init.send_signal(signal.SIGINT)
                init.wait(timeout=5)
            log.close();node.destroy_node()
        manifest=json.loads((WS/'run_data'/run_id/'manifest.json').read_text())
        checks['sim_exit_zero']=manifest['exit_code']==0
        result=dict(mode=mode,run_id=run_id,graph=graph,tf_authorities=auth,checks=checks,
                    code_sha=manifest['code_sha'],verdict='PASS' if all(checks.values()) else 'FAIL')
        results.append(result);print(mode,result['verdict'],checks,flush=True)
finally:
    rclpy.shutdown();dds.send_signal(signal.SIGINT);dds.wait(timeout=10);dlog.close()
report=dict(completed_utc=datetime.now(timezone.utc).isoformat(),modes=results,
            verdict='PASS' if all(r['verdict']=='PASS' for r in results) and len(results)==2 else 'FAIL')
(OUT/'mode-regression.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['verdict']=='PASS' else 1)

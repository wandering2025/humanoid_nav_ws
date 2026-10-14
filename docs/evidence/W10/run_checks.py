"""Actual ROS/Nav2/physical walking fault tests; no navigation action claim."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters

WS=Path(os.environ['T1_WS']); OUT=WS/'docs/evidence/W10'; RAW=OUT/'raw'; RAW.mkdir(exist_ok=True)


class Probe(Node):
    def __init__(self, source, relay):
        super().__init__('w10_probe')
        self.sim=0.; self.state=None; self.gate=None; self.cm=None
        self.rows=[]; self.events=[]; self.block=set(); self.last_record=-1.
        self.pub=self.create_publisher(Twist,'/t1/cmd_'+source,1)
        self.cmds={}
        self.create_subscription(Clock,'/clock',lambda m:setattr(self,'sim',m.clock.sec+m.clock.nanosec*1e-9),100)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String,'/t1/sim_state',lambda m:setattr(self,'state',json.loads(m.data)),qos)
        self.create_subscription(String,'/t1/safety/state',self.safety,qos)
        self.create_subscription(CollisionMonitorState,'/collision_monitor_state',self.monitor,10)
        for topic in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel'):
            self.create_subscription(Twist,'/'+topic,lambda m,t=topic:self.cmds.update({t:[m.linear.x,m.angular.z]}),10)
        self.create_subscription(Odometry,'/w10/raw_odom' if relay else '/odom',self.odom,100)
        if relay:
            for topic,kind in (('scan',LaserScan),('odom',Odometry),('tf',TFMessage)):
                pub=self.create_publisher(kind,'/'+topic,10)
                self.create_subscription(kind,'/w10/raw_'+topic,
                    lambda m,t=topic,p=pub:p.publish(m) if t not in self.block else None,100)

    def safety(self,m):
        data=json.loads(m.data)
        if self.gate is None or (data['state'],data['reason']) != (self.gate['state'],self.gate['reason']):
            self.events.append(dict(kind='gate',sim=self.sim,wall=time.monotonic(),data=data))
        self.gate=data

    def monitor(self,m):
        data=dict(action_type=m.action_type,polygon_name=m.polygon_name)
        if data != self.cm:
            self.events.append(dict(kind='cm',sim=self.sim,wall=time.monotonic(),data=data))
        self.cm=data

    def odom(self,m):
        if self.sim-self.last_record < .099:
            return
        self.last_record=self.sim
        p=m.pose.pose.position; v=m.twist.twist
        self.rows.append(dict(sim=self.sim,wall=time.monotonic(),x=p.x,y=p.y,
            vx=v.linear.x,vy=v.linear.y,wz=v.angular.z,cmds=dict(self.cmds),
            gate=self.gate['state'] if self.gate else None))


class Run:
    def __init__(self, case):
        self.case=case; self.run_id='w10_'+case+'_'+uuid.uuid4().hex[:8]
        self.relay=case not in ('normal','wall','deadband_wall')
        self.node=Probe('navigation' if case=='cancel' else 'teleop',self.relay)
        launch=str(OUT/'fault.launch.py') if self.relay else 'simulation.launch.py'
        self.command=['ros2','launch']+([] if self.relay else ['t1_bringup'])+[launch,
            'run_id:='+self.run_id,'scene:='+str(OUT/('wall.yaml' if case in ('wall','deadband_wall') else 'open.yaml')),
            'target_rtf:=1.0','max_sim_time:=90.0',
            'command_source:='+('navigation' if case=='cancel' else 'teleop')]
        self.log=(RAW/(self.run_id+'.txt')).open('w')
        self.proc=subprocess.Popen(self.command,cwd=WS,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        self.next_pub=0.; self.source=(0.,0.); self.injection=None; self.result={}
        self.started=datetime.now(timezone.utc).isoformat()
        self.children=[]
        print('RUN',case,self.run_id,'PID',self.proc.pid,flush=True)

    def spin(self):
        assert self.proc.poll() is None, 'launch exited: '+self.run_id
        rclpy.spin_once(self.node,timeout_sec=.004)
        if self.source is not None and time.monotonic()>=self.next_pub:
            m=Twist();m.linear.x,m.angular.z=self.source
            self.node.pub.publish(m);self.next_pub=time.monotonic()+.02

    def until(self,predicate,timeout=25.):
        end=time.monotonic()+timeout
        while not predicate():
            assert time.monotonic()<end, f'deadline {self.case} {self.node.gate}'
            self.spin()

    def duration(self,s):
        end=self.node.sim+s
        self.until(lambda:self.node.sim>=end,s*2+10)

    def wall_duration(self,s):
        end=time.monotonic()+s
        self.until(lambda:time.monotonic()>=end,s+5)

    def call(self,kind,name,request):
        client=self.node.create_client(kind,name)
        self.until(client.service_is_ready)
        if self.injection is not None and (name=='/t1/safety/stop' or (name=='/t1/sim/pause' and request.data)):
            self.injection.update(sim=self.node.sim,wall=time.monotonic(),pose=self.node.rows[-1])
        future=client.call_async(request)
        self.until(future.done,10.)
        result=future.result();self.node.destroy_client(client)
        return result

    def arm(self):
        self.source=(0.,0.)
        self.until(lambda:self.node.gate and not self.node.gate['health_reason'])
        self.duration(.7)
        result=self.call(Trigger,'/t1/safety/arm',Trigger.Request())
        assert result.success,result.message
        self.until(lambda:self.node.gate['state']=='ARMED')

    def inject(self):
        self.injection=dict(sim=self.node.sim,wall=time.monotonic(),state=self.node.state,
                            pose=self.node.rows[-1],cm=self.node.cm)
        c=self.case
        if c=='source_timeout':self.source=None
        elif c=='nan':self.source=(float('nan'),0.)
        elif c in ('scan','odom','tf'):self.node.block.add(c)
        elif c=='pause':
            assert self.call(SetBool,'/t1/sim/pause',SetBool.Request(data=True)).success
            self.wall_duration(.7)
            assert self.call(SetBool,'/t1/sim/pause',SetBool.Request(data=False)).success
        elif c=='cancel':
            assert self.call(Trigger,'/t1/safety/stop',Trigger.Request()).success
        elif c in ('monitor_exit','gate_exit'):
            needle='/nav2_collision_monitor/collision_monitor' if c=='monitor_exit' else '/t1_bringup/command_gate'
            table=subprocess.check_output(['ps','-eo','pid=,pgid=,args='],text=True)
            matches=[int(line.split(None,2)[0]) for line in table.splitlines()
                     if len(line.split(None,2))==3 and int(line.split(None,2)[1])==self.proc.pid and needle in line.split(None,2)[2]]
            assert len(matches)==1,matches
            self.result['killed_pid']=matches[0]
            os.kill(matches[0],signal.SIGINT)
        elif c in ('normal','limits'):self.source=(0.,0.)

    def graph(self):
        return {t:[dict(node=i.node_name,type=i.topic_type) for i in self.node.get_publishers_info_by_topic(t)]
                for t in ('/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel','/clock','/scan','/odom')}

    def parameters(self):
        result={}
        for name,names in (
            ('velocity_smoother',['enable_stamped_cmd_vel','use_sim_time','feedback','smoothing_frequency','velocity_timeout','max_velocity','max_accel','max_decel']),
            ('collision_monitor',['enable_stamped_cmd_vel','use_sim_time','base_frame_id','source_timeout','transform_tolerance','Stop.radius','Slow.radius','Slow.slowdown_ratio'])):
            r=self.call(GetParameters,'/'+name+'/get_parameters',GetParameters.Request(names=names))
            result[name]={k:dict(type=v.type,string=v.string_value,boolean=v.bool_value,double=v.double_value,array=list(v.double_array_value)) for k,v in zip(names,r.values)}
        return result

    def execute(self):
        self.until(lambda:self.node.state and self.node.state['state']=='READY')
        self.arm()
        self.result['graph']=self.graph()
        if self.case=='normal':self.result['parameters']=self.parameters()
        self.source=(20.,20.) if self.case=='limits' else (.75 if self.case=='wall' else .5,0.)
        if self.case in ('wall','deadband_wall'):
            self.until(lambda:any(e['kind']=='cm' and e['data']['polygon_name']=='Slow' for e in self.node.events),30.)
            self.result['slow_at']=dict(sim=self.node.sim,wall=time.monotonic())
            if self.case=='wall':
                self.until(lambda:any(e['kind']=='cm' and e['data']['polygon_name']=='Stop' for e in self.node.events),20.)
            self.injection=dict(sim=self.node.sim,wall=time.monotonic(),pose=self.node.rows[-1],cm=self.node.cm)
        else:
            self.duration(5.)
            self.inject()
        self.duration(8.)
        self.result['post_fault_gate']=self.node.gate
        self.result['events']=self.node.events
        if self.case in ('scan','odom','tf','source_timeout','nan','pause','cancel'):
            self.node.block.clear();self.source=(.5,0.)
            self.duration(1.)  # new nonzero alone must NOT clear a latched fault
            self.result['still_latched']=self.node.gate['state']=='STOPPED'
            self.arm();self.source=(.5,0.);self.duration(3.)
            self.result['rearmed']=self.node.gate['state']=='ARMED'
        if self.case=='normal':
            self.source=(0.,.6);self.duration(4.)
            self.source=(0.,0.);self.duration(5.)
            self.source=(0.,-.6);self.duration(4.)
            self.source=(0.,0.);self.duration(5.)
        self.source=(0.,0.)
        self.duration(4.)

    def close(self):
        if self.proc.poll() is None:
            table=subprocess.check_output(['ps','-eo','pid=,pgid='],text=True)
            self.children=[int(line.split()[0]) for line in table.splitlines() if int(line.split()[1])==self.proc.pid]
            os.kill(self.proc.pid,signal.SIGINT)
            try:self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid,signal.SIGTERM);self.proc.wait(timeout=10)
        self.log.close();self.node.destroy_node()

    def analyze(self):
        path=WS/'run_data'/self.run_id
        manifest=json.loads((path/'manifest.json').read_text())
        with (path/'sim_trace.csv').open() as f: rows=list(csv.DictReader(f))
        t=np.array([float(r['sim_time']) for r in rows])
        get=lambda k:np.array([float(r[k]) for r in rows])
        x,y=get('x_w'),get('y_w'); vx,vy,wz=get('v_meas_x'),get('v_meas_y'),get('v_meas_wz')
        u=np.array([[float(r[k]) for k in ('u_pol_x','u_pol_y','u_pol_wz')] for r in rows])
        tinj=self.injection['sim']; idx=int(np.searchsorted(t,tinj)); end=np.searchsorted(t,tinj+7.9)
        zero=np.where(np.all(u[idx:end]==0,axis=1))[0]
        iz=idx+int(zero[0]) if len(zero) else end-1
        tail=(t>tinj+5)&(t<tinj+7.5)
        distance=float(np.max(np.hypot(x[iz:end]-x[iz],y[iz:end]-y[iz])))
        total=float(np.max(np.hypot(x[idx:end]-x[idx],y[idx:end]-y[idx])))
        final_v=float(np.linalg.norm([vx[tail].mean(),vy[tail].mean()]));final_w=float(abs(wz[tail].mean()))
        move=(t>tinj-2)&(t<tinj-.1)
        checks=dict(exit_zero=manifest['exit_code']==0,continued_standing=all(r['state'] not in ('FAULT','FALLEN') for r in rows),
            moved_before_fault=bool(np.max(np.abs(u[move]))>.1),
            zero_reached=bool(len(zero)),no_motion_command_after_stop=bool(np.all(u[iz:end]==0)),
            standing_final=bool(final_v<.05 and final_w<.05),stop_distance=distance<=.353+.10,
            unique_final=self.result['graph']['/cmd_vel']==[{'node':'collision_monitor','type':'geometry_msgs/msg/Twist'}],
            no_speed_boost=all(abs(float(r['v_guard_'+a]))<=abs(float(r['v_cmd_in_'+a]))+1e-9 for r in rows for a in ('x','y','wz')))
        if self.case in ('wall','deadband_wall'):
            checks['no_wall_contact']=bool(np.max(x)<5-.69527)
            checks['slow_observed']=any(e['kind']=='cm' and e['data']['polygon_name']=='Slow' for e in self.node.events)
        if self.case=='wall':checks['stop_observed']=any(e['kind']=='cm' and e['data']['polygon_name']=='Stop' for e in self.node.events)
        if self.case=='deadband_wall':
            checks['filtered_low_speed_zero']=any(.01<float(r['v_cmd_in_x'])<.315 and float(r['u_pol_x'])==0 for r in rows)
        if 'still_latched' in self.result:
            checks['requires_explicit_rearm']=self.result['still_latched'] and self.result['rearmed']
        checks['own_children_exited']=all(not Path('/proc/'+str(pid)).exists() for pid in self.children)
        self.result.update(case=self.case,run_id=self.run_id,started_utc=self.started,ended_utc=datetime.now(timezone.utc).isoformat(),
            tested_sha=manifest['code_sha'],command=self.command,injection=self.injection,
            zero_sim=float(t[iz]),zero_delay_sim=float(t[iz]-tinj),stop_distance_from_zero_m=distance,
            distance_from_injection_m=total,final_speed_mps=final_v,final_yaw_speed=final_w,
            scan_compute_p95_ms=float(np.percentile([float(r['scan_compute_ms']) for r in rows if r['scan_due']=='True'],95)),
            checks={k:bool(v) for k,v in checks.items()},verdict='PASS' if all(checks.values()) else 'FAIL',
            launch_exit=self.proc.returncode,manifest=str(path/'manifest.json'),children=self.children,
            timeline=self.node.rows[::2])
        (OUT/(self.run_id+'.json')).write_text(json.dumps(self.result,indent=2,allow_nan=False)+'\n')
        print('RESULT',self.case,self.result['verdict'],checks,'distance',distance,'zero delay',t[iz]-tinj,flush=True)
        return self.result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('cases',nargs='*',default=['normal'])
    args=parser.parse_args()
    dlog=(RAW/'dds.txt').open('a');dds=subprocess.Popen(['bash',str(WS/'scripts/start_local_dds.sh')],stdout=dlog,stderr=subprocess.STDOUT)
    results=[]
    try:
        time.sleep(.7);assert dds.poll() is None
        rclpy.init()
        for case in args.cases:
            run=Run(case)
            try:run.execute()
            except Exception as exc:
                run.result['error']=repr(exc)
                (OUT/(run.run_id+'-error.json')).write_text(json.dumps(run.result,indent=2)+'\n')
                raise
            finally:run.close()
            results.append(run.analyze())
        return 0 if all(r['verdict']=='PASS' for r in results) else 1
    finally:
        if rclpy.ok():rclpy.shutdown()
        dds.send_signal(signal.SIGINT);dds.wait(timeout=10);dlog.close()


if __name__=='__main__':raise SystemExit(main())

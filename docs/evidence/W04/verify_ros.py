"""W04 acceptance driver. Uses installed executable; never writes physical state."""
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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

WS=Path(os.environ['T1_WS'])
OUT=WS/'docs/evidence/W04'
EXE=WS/'install/t1_mujoco/lib/t1_mujoco/t1_sim'


def trace_summary(path):
    manifest=json.loads((path/'manifest.json').read_text())
    with (path/'sim_trace.csv').open() as f: rows=list(csv.DictReader(f))
    times=[float(r['sim_time']) for r in rows]
    assert rows and all(b>a for a,b in zip(times,times[1:]))
    for row in rows:
        for key in ('x_w','y_w','z_w','yaw_w','proj_grav_z','action_norm'):
            assert math.isfinite(float(row[key]))
        assert float(row['proj_grav_z']) <= -.5
        assert row['nonfinite']=='False'
    return dict(run_id=path.name,manifest=manifest,rows=len(rows),sim_time_end=times[-1],
                min_height=min(float(r['z_w']) for r in rows),
                max_proj_grav_z=max(float(r['proj_grav_z']) for r in rows),
                first_wall_mono=float(rows[0]['wall_mono']),last_wall_mono=float(rows[-1]['wall_mono']),
                control_wall_duration=float(rows[-1]['wall_mono'])-float(rows[0]['wall_mono']),
                control_rtf=(times[-1]-times[0])/(float(rows[-1]['wall_mono'])-float(rows[0]['wall_mono'])),
                final_position=[float(rows[-1][k]) for k in ('x_w','y_w','z_w')])


class Driver:
    def __init__(self):
        self.node=Node('w04_acceptance')
        self.latest={};self.states=[];self.clocks=[];self.process=None;self.run_id=''
        self.node.create_subscription(String,'/t1/sim_state',self.state,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(Clock,'/clock',lambda m:self.clocks.append(m.clock.sec+m.clock.nanosec/1e9),10000)
        self.pub=self.node.create_publisher(Twist,'/t1/debug/policy_cmd',1)
        self.physical=self.node.create_publisher(Twist,'/cmd_vel',1)
        self.pause=self.node.create_client(SetBool,'/t1/sim/pause')
        self.reset=self.node.create_client(Trigger,'/t1/sim/reset')

    def state(self,msg):
        value=json.loads(msg.data)
        if value.get('run_id')==self.run_id:
            self.latest=value;self.states.append(value)

    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=min(.02,max(0,end-time.monotonic())))

    def until(self,predicate,timeout=12):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.spin(.02)
            if predicate():return
            if self.process.poll() is not None:
                raise AssertionError(f'child exited {self.process.returncode}; log {self.path}/stdout.txt')
        raise AssertionError(f'timeout; latest={self.latest}; nodes={self.node.get_node_names()}; state_publishers={self.node.get_publishers_info_by_topic("/t1/sim_state")}; clocks={len(self.clocks)}')

    def start(self,case,params):
        self.run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_dev_'+case+'_s0_'+uuid.uuid4().hex[:6]
        self.path=WS/'run_data'/self.run_id
        # Runtime owns this directory; stdout lives in separate ignored raw area.
        raw=OUT/'raw';raw.mkdir(exist_ok=True)
        self.stdout=(raw/(self.run_id+'.txt')).open('w')
        self.latest={};self.states=[];self.clocks=[]
        command=[str(EXE),'--ros-args','-p','run_id:='+self.run_id]
        for name,value in params.items():command+=['-p',name+':='+str(value).lower()]
        self.command=command
        env=dict(os.environ)
        if params.get('headless',True):
            env.pop('DISPLAY',None);env.pop('WAYLAND_DISPLAY',None)
        self.process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=self.stdout,stderr=subprocess.STDOUT,env=env,cwd='/tmp')
        self.until(lambda:self.latest.get('state')=='READY')
        assert self.node.count_publishers('/clock')==1

    def send(self,x=.5,*,publisher=None,**components):
        msg=Twist();msg.linear.x=x
        for name,value in components.items():
            vector,axis=name.split('_');setattr(getattr(msg,vector),axis,value)
        (publisher or self.pub).publish(msg)

    def stream(self,duration,x=.5):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            self.send(x);self.spin(.04)

    def service(self,client,request):
        assert client.wait_for_service(timeout_sec=3.)
        future=client.call_async(request)
        self.until(future.done,4)
        assert future.result().success,future.result().message
        return future.result().message

    def finish(self,sig=signal.SIGTERM):
        self.process.send_signal(sig)
        code=self.process.wait(timeout=10)
        self.spin(.1)
        self.stdout.close()
        assert code==0,code
        result=trace_summary(self.path)
        assert result['manifest']['executable']==os.environ['T1_VENV']+'/bin/python'
        assert all(b>=a for a,b in zip(self.clocks,self.clocks[1:]))
        result.update(exit_code=code,command=self.command,clock_messages=len(self.clocks),
                      clock_monotonic=True,events=self.states,child_pid=self.process.pid)
        self.process=None
        return result

    def cleanup(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.process.kill();self.process.wait()
        self.node.destroy_node()


def interaction(d):
    d.start('controls',{'enable_policy_cmd_input':True})
    assert d.node.count_subscribers('/cmd_vel')==0
    d.stream(1)
    d.until(lambda:d.latest.get('cmd_status')=='ACTIVE')
    assert d.latest['u_pol']==[.5,0.,0.]
    d.until(lambda:d.latest.get('cmd_status')=='TIMEOUT')
    assert d.latest['u_pol']==[0.,0.,0.]
    for bad in (float('nan'),float('inf')):
        d.send(bad)
        d.until(lambda:d.latest.get('cmd_status')=='INVALID')
        assert d.latest['u_pol']==[0.,0.,0.]
        d.stream(.2)
        d.until(lambda:d.latest.get('cmd_status')=='ACTIVE')
    d.send(9.)
    d.spin(.1)
    # A status heartbeat may not occur before timeout; trace verifies bound.
    d.stream(.3)
    d.service(d.pause,SetBool.Request(data=True))
    d.until(lambda:d.latest.get('state')=='PAUSED')
    d.spin(.1);clock_count=len(d.clocks);paused_time=d.latest['sim_time']
    d.stream(.7)
    assert len(d.clocks)==clock_count and d.latest['sim_time']==paused_time
    d.service(d.pause,SetBool.Request(data=False))
    d.until(lambda:d.latest.get('state')=='READY')
    d.spin(.2)
    assert d.latest['u_pol']==[0.,0.,0.]
    d.stream(.3)
    old_episode=d.latest['episode'];old_clock=d.clocks[-1]
    d.service(d.reset,Trigger.Request())
    d.until(lambda:d.latest.get('episode')==old_episode+1 and d.latest.get('state')=='READY')
    assert d.clocks[-1]>old_clock and d.latest['u_pol']==[0.,0.,0.]
    d.spin(.2)
    result=d.finish()
    result['checks']=['continuous_input','wall_timeout','nan_inf','valid_after_invalid','debug_clamp',
                      'pause_clock_frozen','pause_discards_input','resume_no_stale_command','reset_monotonic_new_episode','SIGTERM','closed_stdin','clock_unique']
    with (d.path/'sim_trace.csv').open() as f: rows=list(csv.DictReader(f))
    assert max(float(row['u_pol_x']) for row in rows)==1.
    assert any(row['cmd_status']=='INVALID' for row in rows)
    assert any(row['cmd_status']=='TIMEOUT' for row in rows)
    return result


def uncalibrated(d):
    d.start('uncalibrated',{})
    assert d.node.count_subscribers('/t1/debug/policy_cmd')==0
    d.send(.5,publisher=d.physical)
    d.spin(.1)
    d.until(lambda:d.latest.get('cmd_status')=='UNCALIBRATED')
    assert d.latest['u_pol']==[0.,0.,0.]
    d.send(float('nan'),publisher=d.physical)
    d.until(lambda:d.latest.get('cmd_status')=='INVALID')
    d.send(.5,publisher=d.physical)
    d.until(lambda:d.latest.get('cmd_status')=='UNCALIBRATED')
    d.until(lambda:d.latest.get('cmd_status')=='TIMEOUT')
    return d.finish(signal.SIGINT)


def standing():
    results=[]
    for i in range(3):
        run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+f'_dev_standing{i}_s0_'+uuid.uuid4().hex[:6]
        command=[str(EXE),'--ros-args','-p','target_rtf:=0.0','-p','max_sim_time:=60.0','-p','run_id:='+run_id]
        env=dict(os.environ);env.pop('DISPLAY',None);env.pop('WAYLAND_DISPLAY',None)
        start=time.monotonic()
        p=subprocess.run(command,cwd='/tmp',stdin=subprocess.DEVNULL,capture_output=True,text=True,env=env,timeout=30)
        assert p.returncode==0,p.stdout+p.stderr
        result=trace_summary(WS/'run_data'/run_id)
        assert abs(result['sim_time_end']-60.)<1e-8 and result['rows']==3000
        result.update(exit_code=p.returncode,process_wall_duration=time.monotonic()-start,command=command)
        results.append(result)
    assert len({tuple(r['final_position']) for r in results})==1
    return results


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('suite',choices=['controls','standing'])
    suite=parser.parse_args().suite
    started=datetime.now(timezone.utc).isoformat();rclpy.init();d=Driver()
    output=dict(suite=suite,started_utc=started,code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip())
    try:
        if suite=='controls': output['results']=[interaction(d),uncalibrated(d)]
        elif suite=='standing':output['results']=standing()
        output['verdict']='PASS'
    except Exception as exc:
        output['verdict']='FAIL';output['failure']=repr(exc)
        raise
    finally:
        d.cleanup();rclpy.shutdown()
        output['ended_utc']=datetime.now(timezone.utc).isoformat()
        (OUT/(suite+'-results.json')).write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in output.items() if k!='results'}))

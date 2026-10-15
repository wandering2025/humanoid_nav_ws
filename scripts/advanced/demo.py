"""Independent v0.2 orchestrator; reuse production process/stop/render ownership."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

WS = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(WS/'scripts/demo'), str(WS/'scripts')]
import demo as base
OUT, RUN_ID = base.OUT, base.RUN_ID


def preflight():
    import yaml
    import mujoco
    from ament_index_python.packages import get_package_prefix
    assert os.environ['ROS_DISTRO'] == 'jazzy'
    assert Path(sys.prefix).resolve() == Path(os.environ['T1_VENV']).resolve()
    assert mujoco.__version__ == '3.15.0'
    for name in ('t1_mujoco','t1_bringup','t1_description','t1_navigation'):
        assert Path(get_package_prefix(name)).is_relative_to(WS/'install'), name
    assert base.git('rev-parse','demo-v0.1.0^{}') == '0f51d54b1dbfdde0664966118b5546109a3f6ff2'
    for name in ('scripts/run_demo.sh','src/t1_navigation/config/safety.yaml',
                 'src/t1_navigation/config/navigation.yaml','src/t1_mujoco/config/command_map.yaml',
                 'src/t1_bringup/t1_bringup/navigation_adapter.py',
                 'src/t1_bringup/t1_bringup/navigation_tools.py',
                 'src/t1_bringup/t1_bringup/safety.py','src/t1_mujoco/t1_mujoco/runner.py'):
        original=subprocess.check_output(['git','show','demo-v0.1.0:'+name],cwd=WS)
        assert (WS/name).read_bytes()==original, name
    for path, expected in ((os.environ['T1_UPSTREAM'],'7bb1462e2ac742eba5c01659b815e2e99fbe617f'),
                           (os.environ['T1_ASSETS'],'12a97516f57469078707afb587a5f84c1cb110d1')):
        assert base.command(['git','-C',path,'rev-parse','HEAD'])==expected
        assert not base.command(['git','-C',path,'status','--porcelain'])
    assert not base.project_processes(), 'Existing demo processes; do not disturb them'
    assert base.port_free(), 'DDS port already occupied'
    return dict(code_sha=base.git('rev-parse','HEAD'),dirty=base.git('status','--porcelain'),
                stable_tag=base.git('rev-parse','demo-v0.1.0'),cwd=str(WS))


class Advanced(base.Demo):
    def __init__(self,args,provenance):
        super().__init__(args,provenance)
        self.summary.update(scenario='ADV01',method='guided_autonomous_mapping',goals=[],map_growth=[])
        self.route=__import__('yaml').safe_load((WS/'src/t1_bringup/config/advanced_route.yaml').read_text())
        self.runtime_graph={}

    def markers(self):
        if not hasattr(self,'viz'): return
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        a=MarkerArray()
        for i,(x,label,color) in enumerate(((0.,'A START',(.1,.75,.95)),(14.,'B GOAL',(.2,.95,.45)))):
            m=Marker();m.header.frame_id='map';m.ns='route';m.id=2*i
            m.type=Marker.LINE_STRIP;m.pose.orientation.w=1.;m.scale.x=.06
            m.color.r,m.color.g,m.color.b=color;m.color.a=1.
            m.points=[Point(x=x+.65*math.cos(j*math.pi/40),y=.65*math.sin(j*math.pi/40),z=.06) for j in range(81)]
            a.markers.append(m)
            m=Marker();m.header.frame_id='map';m.ns='route';m.id=2*i+1
            m.type=Marker.TEXT_VIEW_FACING;m.pose.orientation.w=1.;m.pose.position.x=x
            m.pose.position.y=-1.;m.scale.z=.5;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;m.text=label
            a.markers.append(m)
        m=Marker();m.header.frame_id='map';m.ns='phase';m.id=0;m.type=Marker.TEXT_VIEW_FACING
        m.pose.orientation.w=1.;m.pose.position.x=7.;m.pose.position.y=13.;m.scale.z=.5
        m.color.r=m.color.g=m.color.b=1.;m.color.a=1.;m.text=self.summary['status']
        a.markers.append(m);self.viz.publish(a)

    def on_map(self,m):
        known=sum(v>=0 for v in m.data)
        occupied=sum(v>=65 for v in m.data)
        self.summary['map_growth'].append(dict(sim=self.node.sim,known_cells=known,occupied_cells=occupied,
            known_m2=known*m.info.resolution**2,width=m.info.width,height=m.info.height,
            resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y]))

    def save_map(self,label):
        target=OUT/'maps'/label;target.parent.mkdir(exist_ok=True)
        p=self.spawn('map_saver',['ros2','run','nav2_map_server','map_saver_cli','-f',str(target),
             '--ros-args','-p','use_sim_time:=true','-p','save_map_timeout:=10.0'])
        self.until(lambda:p.poll() is not None,25)
        assert p.returncode==0, 'map_saver failed'
        self.children['map_saver'][1].close()
        (OUT/'map_saver.log').rename(OUT/f'map_saver-{label}.log')
        return {str(p.relative_to(OUT)):base.sha(p) for p in target.parent.glob(label+'.*')}

    def goal(self,pose,label,limit):
        self.summary['status']=label
        self.markers()
        p=self.spawn('goal',['ros2','run','t1_bringup','navigate_to_pose',*map(str,pose),
                            '--time-limit',str(limit)])
        self.until(lambda:p.poll() is not None,limit*2.5+100)
        lines=[json.loads(l) for l in self.goal_log.read_text().splitlines() if l.startswith('{')]
        nav=next((l for l in reversed(lines) if l.get('event')=='result'),{})
        self.children['goal'][1].close()
        self.goal_log.rename(OUT/f'{label}.log')
        self.summary['goals'].append(dict(label=label,pose=pose,exit_code=p.returncode,result=nav))
        base.save('partial-result.json',self.summary)
        assert p.returncode==0 and nav.get('verified') and nav.get('status')=='SUCCEEDED', f'{label} failed: {nav}'
        return nav

    def execute(self):
        os.environ['T1_EVAL_OUT']=str(OUT);os.environ['T1_DEMO_PRESENTATION']='1'
        from evaluation.harness import Probe
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid
        from visualization_msgs.msg import MarkerArray
        from std_srvs.srv import Trigger
        from lifecycle_msgs.srv import GetState
        self.start_gpu_watch()
        self.spawn('dds',[str(WS/'scripts/start_local_dds.sh')])
        self.until(lambda:'### Server is running ###' in (OUT/'dds.log').read_text(errors='replace')
                   and bool(base.port_connections()['listeners']),10)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node=Probe()
        from grid_visuals import GridVisuals
        self.grid_visuals=GridVisuals(self.node)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.viz=self.node.create_publisher(MarkerArray,'/t1/viz/demo',qos)
        self.node.create_subscription(OccupancyGrid,'/map',self.on_map,qos)
        self.action=ActionClient(self.node,NavigateToPose,'navigate_to_pose')
        self.spawn('launch',['ros2','launch','t1_bringup','advanced.launch.py',
            f'headless:={str(self.args.headless).lower()}','rviz:=false',f'run_id:={RUN_ID}',
            'seed:=4000','target_rtf:=1.0','max_sim_time:=1000',
            f'params_file:={WS}/src/t1_navigation/config/w12_mapping_runtime.yaml',
            f'events_file:={WS}/src/t1_bringup/config/advanced_events.yaml','scenario:=ADV01'])
        self.summary['status']='SLAM_INITIALIZING'
        self.until(lambda:'"starter": "navigation active"' in self.launch_log.read_text(errors='replace'),120)
        n=self.node
        self.until(lambda:n.state and n.state['state']=='READY' and n.map and n.map_pose() and n.scan
                   and n.gate and not n.gate.get('health_reason') and self.action.server_is_ready(),30)
        lifecycle={name:self.call(GetState,f'/{name}/get_state',GetState.Request()).current_state.id
                   for name in ('slam_toolbox','controller_server','planner_server','behavior_server',
                                'bt_navigator','velocity_smoother','collision_monitor')}
        assert all(v==3 for v in lifecycle.values()),lifecycle
        assert not n.initializations, 'Live SLAM must not use AMCL initialpose'
        self.summary.update(lifecycle=lifecycle,readiness_wall_s=time.monotonic()-self.started)
        if not self.args.headless and not self.args.no_rviz:
            os.environ['T1_RVIZ_HEARTBEAT']=str(OUT/'rviz-heartbeat')
            self.spawn('rviz',[str(WS/'build/demo-rviz/t1_demo_rviz'),'-d',str(WS/'src/t1_navigation/config/advanced.rviz'),
                             '--ros-args','-p','use_sim_time:=true'])
            self.until(lambda:(OUT/'rviz-heartbeat').exists() and 'RVIZ_LIFECYCLE ready' in (OUT/'rviz.log').read_text(errors='replace'),25)
            from gpu_watch import renderer_verified
            assert renderer_verified((OUT/'rviz.log').read_text()), 'Unexpected renderer'
            until=n.sim+2;self.until(lambda:n.sim>=until,10)
        self.markers();self.screenshot('ready')
        if self.args.startup_check:
            end=n.sim+30;self.until(lambda:n.sim>=end,60)
            self.safe_stop();self.summary['startup_check_completed']=True
            return
        self.summary['initial_map']=self.save_map('initial')
        for index,pose in enumerate(self.route['mapping_waypoints']):
            self.goal(pose,f'MAPPING_{index+1:02d}',self.route['waypoint_time_limit_s'])
        assert n.obstacle and n.obstacle['triggered_at'] is None, 'Obstacle moved during mapping'
        self.summary['status']='MAPPING_COMPLETE_AT_A'
        self.summary['mapping_completed_sim']=n.sim
        self.summary['generated_map']=self.save_map('mapped')
        self.screenshot('mapped')
        # Service changes only the experiment source, never a navigation command.
        response=self.call(Trigger,'/t1/advanced/enable_obstacle',Trigger.Request())
        assert response.success
        self.until(lambda:n.obstacle and n.obstacle.get('enabled'),5)
        self.summary['navigation_start_sim']=n.sim
        self.summary['nav2']=self.goal(self.route['goal'],'NAVIGATING_A_TO_B',self.route['navigation_time_limit_s'])
        end=n.sim+12;self.until(lambda:n.sim>=end,35)
        self.summary['final_map']=self.save_map('final')
        self.screenshot('arrived');self.safe_stop()
        self.summary['status']='ARRIVED_SAFE_STOP'
        self.markers();base.save('partial-result.json',self.summary)
        if not self.args.headless:
            end=time.monotonic()+8
            while not base.STOP and (self.args.hold or time.monotonic()<end):self.spin(False)

    def metrics(self):
        import numpy as np
        from evaluation.episode import load_trace,settle_analysis
        from evaluation.metrics import body_clearance
        artifact=WS/'run_data'/RUN_ID
        manifest=json.loads((artifact/'manifest.json').read_text())
        trace=load_trace(RUN_ID);n=self.node;s=self.summary
        s.update(contacts=manifest.get('contact_counts'),fell=any(v in ('FALLEN','FAULT') for v in trace['state']),
                 simulator_exit_code=manifest.get('exit_code'),body_clearance=body_clearance(manifest,artifact/'geometry.jsonl'))
        if self.args.startup_check:
            s['arrived']=False
            s['zero_policy']=bool(np.all(trace['u_pol_x']==0) and np.all(trace['u_pol_wz']==0))
            behavior=s.get('startup_check_completed') and s['zero_policy']
        else:
            nav=s.get('nav2',{})
            s['settle']=settle_analysis(trace,self.route['goal'],nav.get('nav2_success_sim',trace['sim_time'][-1]),
                                       dict(position_m=.3,yaw_deg=20),2.)
            s['arrived']=bool(nav.get('verified') and s['settle']['settled'])
            start=s.get('navigation_start_sim',math.inf)
            plans=[p for p in n.plans if p['sim']>=start]
            s['navigation_plans']=plans
            s['nonstraight_plan']=bool(plans and plans[0]['length']>19 and max(p[1] for p in plans[0]['points'])>6)
            growth=s['map_growth'];s['map_growth_ratio']=growth[-1]['known_cells']/growth[0]['known_cells'] if growth else 0
            # Measured robot stationarity during the obstacle's active interval.
            active=[o for o in n.obstacles if o['phase'] not in ('staged','finished')]
            hold=best=0.
            if active:
                for i,row in enumerate(n.rows):
                    if not (active[0]['sim_time']<=row['sim']<=active[-1]['sim_time']) or i<5:
                        hold=0;continue
                    old=n.rows[i-5];dt=row['sim']-old['sim']
                    still=(dt>0 and math.hypot(row['x']-old['x'],row['y']-old['y'])/dt<.05
                           and abs(math.atan2(math.sin(row['yaw']-old['yaw']),math.cos(row['yaw']-old['yaw'])))/dt<.05
                           and row['cm'] in (1,2))
                    hold=hold+row['sim']-n.rows[i-1]['sim'] if still else 0.;best=max(best,hold)
            hits=sum(bool(x['hits']) and x['obstacle']['triggered_at'] is not None for x in n.dynamic_scans)
            s['dynamic']=dict(protective_stationary_s=best,scan_hit_frames=hits,
                triggered=bool(active),finished=bool(n.obstacle and n.obstacle['phase']=='finished'))
            s['dynamic']['verdict']='PASS' if best>=.5 and hits>0 and s['dynamic']['finished'] and s['arrived'] else 'FAIL'
            behavior=(s['arrived'] and s['nonstraight_plan'] and s['map_growth_ratio']>1.3
                      and s['dynamic']['verdict']=='PASS' and len(s['goals'])==len(self.route['mapping_waypoints'])+1)
        s['graph_contract']=dict(final_only_collision_monitor=self.runtime_graph.get('/cmd_vel',{}).get('publishers')==['collision_monitor'],
             obstacle_truth_isolated=self.runtime_graph.get('/t1/eval/obstacle',{}).get('subscribers')==['t1_evaluator'],
             robot_truth_isolated=self.runtime_graph.get('/t1/eval/ground_truth',{}).get('subscribers')==['t1_evaluator'],
             no_initialpose=not s.get('initializations'))
        exits=all(v==0 for v in s.get('parent_exit_codes',{}).values()) and bool(s.get('launch_process_exit_codes')) and all(v['exit_code']==0 for v in s['launch_process_exit_codes'].values())
        s['validation_before_port_check']=bool(behavior and exits and s['simulator_exit_code']==0 and not s['fell']
            and s['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0) and all(s['graph_contract'].values())
            and s.get('safe_stop',{}).get('physical_stationary') and not s.get('remaining_owned_pids')
            and not s.get('remaining_project_processes') and s.get('clock_regressions')==0
            and not s.get('error') and not s.get('cleanup_error') and not s.get('gpu_monitor_error'))
        base.save('observation.json',dict(rows=n.rows,obstacles=n.obstacles,plans=n.plans,scan_stats=n.scan_stats))
        s['status']='PENDING_PORT_CHECK' if s['validation_before_port_check'] else 'FAILED'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for flag in ('headless','no-rviz','hold','startup-check','preflight'):p.add_argument('--'+flag,action='store_true')
    args=p.parse_args();args.gpu_watch=None
    (WS/'run_data/demo').mkdir(parents=True,exist_ok=True)
    lock=(WS/'run_data/demo/active.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    signal.signal(signal.SIGINT,lambda *_:setattr(base,'STOP',True))
    signal.signal(signal.SIGTERM,lambda *_:setattr(base,'STOP',True))
    if args.headless or args.preflight:
        from cpu_guard import enter
        base.save('cpu-guard.json',enter())
    provenance=preflight();base.save('provenance.json',provenance)
    if args.preflight:return 0
    if not args.headless:
        from gpu_watch import Watch,GPU_ENV
        os.environ.update(GPU_ENV);args.gpu_watch=Watch(OUT/'gpu',idle_wait_s=60)
    run=Advanced(args,provenance)
    base.say(f'Advanced run {RUN_ID}; {OUT}')
    try:run.execute()
    except Exception as exc:
        run.summary['error']=repr(exc);(OUT/'failure.txt').write_text(traceback.format_exc())
        base.say(f'FAILED: {exc}')
    finally:
        if run.node:
            run.runtime_graph=run.node.graph(('/cmd_vel','/t1/eval/obstacle','/t1/eval/ground_truth','/t1/cmd_teleop','/map','/tf'))
        try:run.cleanup()
        except Exception:run.summary['cleanup_error']=traceback.format_exc()
        try:run.metrics()
        except Exception:run.summary.update(metric_error=traceback.format_exc(),status='FAILED')
        base.save('result.json',run.summary)
    return 0 if run.summary.get('validation_before_port_check') else 1

if __name__=='__main__':raise SystemExit(main())

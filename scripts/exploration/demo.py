"""Own a W16 run and evaluate it separately from the map-only decision process."""
import argparse
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback

WS = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('advanced_demo', WS/'scripts/advanced/demo.py')
advanced = importlib.util.module_from_spec(spec); spec.loader.exec_module(advanced)
base, OUT, RUN_ID = advanced.base, advanced.OUT, advanced.RUN_ID


class ExplorationRun(advanced.Advanced):
    managed_shutdown_order = ('goal','rviz','authority','launch','scan_filter','dds')

    def __init__(self, args, provenance):
        # Do not initialize Advanced's guided route or its scene-specific input.
        base.Demo.__init__(self, args, provenance)
        self.summary.update(scenario='W16', method='goal_directed_frontier_exploration', map_growth=[])
        self.runtime_graph, self.global_grid, self.active_target = {}, None, None
        self.last_visual = 0.
        self.rviz_config = WS/'src/t1_navigation/config/exploration.rviz'
        self.display_start = None
        self.display_target = None
        self.display_phase = 'Waiting for online SLAM'
        self.display_iteration = 0
        self.dynamic_screenshot = False

    def markers(self):
        if not hasattr(self, 'viz'):
            return
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        markers = MarkerArray()
        def marker(ns, kind, color, frame='map'):
            m = Marker(); m.header.frame_id = frame; m.ns = ns; m.id = 0
            m.type = kind; m.pose.orientation.w = 1.
            m.color.r, m.color.g, m.color.b = color; m.color.a = 1.
            markers.markers.append(m)
            return m
        def text(ns, value, xy, color, size=.55, frame='map'):
            m = marker(ns, Marker.TEXT_VIEW_FACING, color, frame)
            m.pose.position.x, m.pose.position.y = xy; m.pose.position.z = .15
            m.scale.z = size; m.text = value
        def line(ns, points, color, width, frame='map'):
            m = marker(ns, Marker.LINE_STRIP, color, frame); m.scale.x = width
            m.points = [Point(x=float(x), y=float(y), z=.12) for x, y in points]
        if self.display_start:
            # RViz applies live map->odom. These are the explorer's actual A/B,
            # not scene coordinates or a guessed initial robot heading.
            for label, xy, color in (('A START', self.display_start[0][:2], (.05,.45,.65)),
                                     ('B GOAL', self.display_start[1], (.05,.5,.22))):
                x, y = xy
                line(label, [(x+.65*math.cos(j*math.pi/40), y+.65*math.sin(j*math.pi/40))
                             for j in range(81)], color, .06, 'odom')
                text(label+' label', label, (x,y-1.), color, frame='odom')
        phase = self.display_phase
        if self.summary['status'] == 'ARRIVED_SAFE_STOP':
            phase = 'B reached - safely stopped'
        elif self.display_target and self.node.odom:
            v = self.node.odom.twist.twist
            if abs(v.linear.x)<.05 and abs(v.angular.z)<.05 and self.node.cmds['final']==(0.,0.):
                phase = 'Waiting / replanning from live observations'
        text('phase', phase, (7.,14.), (.1,.2,.3), .65)
        text('legend', 'Blue: latest Nav2 plan | Orange: selected target | Purple: travelled',
             (7.,13.), (.1,.2,.3), .42)
        if self.display_target:
            x, y = self.display_target
            line('selected_target', [(x-.25,y),(x+.25,y),(x,y),(x,y-.25),(x,y+.25)], (.95,.45,.05), .10)
        trail = [(r['mx'],r['my']) for r in self.node.rows[::3] if r['mx'] is not None]
        if len(trail)>1:
            line('actual_trail', trail, (.5,.18,.65), .035)
        self.viz.publish(markers)

    def on_exploration(self, msg):
        from t1_mujoco.exploration_display import start_markers
        event = json.loads(msg.data)
        start = start_markers(event)
        if start:
            self.display_start = start
            self.summary['display_start'] = dict(anchor_odom=start[0], goal_odom=start[1])
        kind = event.get('event')
        if kind == 'decision':
            self.display_iteration = event['iteration']+1
            self.display_target = tuple(event['target'][:2])
            self.display_phase = ('Known route to B - navigating' if event['final'] else
                                  f'Exploring - observation target {self.display_iteration}')
        elif kind == 'temporarily_blocked':
            self.display_phase = 'Waiting for new observations'
        elif kind == 'frontier_invalidated_by_observation':
            self.display_phase = 'New obstacle observed - selecting another target'

    def launch_command(self):
        commands = super().launch_command()
        commands = [x for x in commands if not x.startswith(('events_file:=', 'scenario:=', 'params_file:='))]
        if not self.args.static:
            commands += [f'events_file:={WS}/src/t1_bringup/config/exploration_events.yaml', 'scenario:=EXP01']
        return commands+[f'scene:={self.args.scene}', f'slam_params_file:={OUT}/slam.yaml',
                         f'params_file:={OUT}/simulation.yaml']

    def execute(self):
        self.spawn('scan_filter', [str(Path(os.environ['T1_VENV'])/'bin/python'),
            str(WS/'scripts/exploration/scan_filter.py'), '--output', str(OUT/'scan-filter.jsonl')])
        super().execute()

    def spin(self, check_stop=True):
        if check_stop and 'scan_filter' in self.children and self.children['scan_filter'][0].poll() is not None:
            raise RuntimeError('mapping scan filter exited unexpectedly')
        super().spin(check_stop)
        if (check_stop and not self.args.headless and not self.dynamic_screenshot
                and self.node and self.node.obstacle and self.node.obstacle['phase'].startswith('hold_')):
            self.dynamic_screenshot = True
            self.screenshot('dynamic')

    def mission(self):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        self.node.create_subscription(String, '/t1/exploration/status', self.on_exploration,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.summary['status'] = 'EXPLORING'
        self.summary['initial_map'] = self.save_map('initial')
        child = self.spawn('goal', [str(Path(os.environ['T1_VENV'])/'bin/python'),
            str(WS/'scripts/exploration/controller.py'), '--task', str(OUT/'task.yaml'),
            '--output', str(OUT/'explorer'), '--time-limit', str(self.args.time_limit)])
        topics = ('/map','/tf','/global_costmap/costmap','/t1/eval/ground_truth','/t1/eval/obstacle',
                  '/cmd_vel','/scan','/scan/mapping')
        def connected():
            graph = self.node.graph(topics)
            self.summary['decision_graph'] = graph
            return (all('t1_explorer' in graph.get(topic,{}).get('subscribers',[])
                        for topic in ('/map','/global_costmap/costmap','/tf'))
                    or child.poll() is not None)
        self.until(connected, 30)
        self.until(lambda: child.poll() is not None, self.args.time_limit*2.5+150)
        result_file = OUT/'explorer/result.json'
        if not result_file.is_file():
            raise RuntimeError(f'explorer exited {child.returncode} before result; see goal.log')
        result = json.loads(result_file.read_text())
        self.summary['exploration'] = result
        self.summary['nav2'] = dict(verified=result['status']=='ARRIVED')
        self.summary['final_map'] = self.save_map('final')
        self.safe_stop()
        if child.returncode != 0 or result['status'] != 'ARRIVED':
            raise RuntimeError(f'Exploration failed: {result}')
        self.summary['status'] = 'ARRIVED_SAFE_STOP'
        self.markers()
        if not self.args.headless:
            # Allow RViz to draw the final status before capturing actual windows.
            ready = self.node.sim+.5
            self.until(lambda: self.node.sim >= ready, 5, False)
            self.screenshot('arrived')
            end = time.monotonic()+8
            while not base.STOP and (self.args.hold or time.monotonic()<end):
                self.spin(False)

    def metrics(self):
        import numpy as np
        from evaluation.episode import load_trace
        artifact = WS/'run_data'/RUN_ID
        manifest = json.loads((artifact/'manifest.json').read_text())
        trace = load_trace(RUN_ID); s = self.summary
        result = s.get('exploration', {})
        s.update(contacts=manifest.get('contact_counts'),
                 fell=any(v in ('FALLEN','FAULT') for v in trace['state']),
                 simulator_exit_code=manifest.get('exit_code'))
        goal = result.get('goal_odom')
        if goal:
            errors = np.hypot(trace['x_w']-goal[0], trace['y_w']-goal[1])
            last = trace['sim_time'] >= trace['sim_time'][-1]-2.
            s['truth_final_error_m'] = float(errors[-1])
            s['truth_final_hold_max_error_m'] = float(np.max(errors[last]))
        s['arrived'] = bool(result.get('status')=='ARRIVED' and s.get('truth_final_hold_max_error_m',math.inf)<=.3)
        authority = [json.loads(l) for l in (OUT/'authority.log').read_text().splitlines() if l.startswith('{')]
        actual = [r for r in authority if r['parent']=='map' and r['child']=='odom']
        s['map_odom_authorities'] = sorted({r['node'] for r in actual if r['node']!='unresolved'})
        s['map_odom_gids'] = sorted({r['gid'] for r in actual})
        graph = s.get('decision_graph', {})
        s['graph_contract'] = dict(
            map_only_slam=graph.get('/map',{}).get('publishers')==['slam_toolbox'],
            explorer_reads_map='t1_explorer' in graph.get('/map',{}).get('subscribers',[]),
            explorer_reads_live_costmap='t1_explorer' in graph.get('/global_costmap/costmap',{}).get('subscribers',[]),
            truth_isolated=graph.get('/t1/eval/ground_truth',{}).get('subscribers')==['t1_evaluator'],
            obstacle_truth_isolated=graph.get('/t1/eval/obstacle',{}).get('subscribers')==['t1_evaluator'],
            final_only_collision_monitor=graph.get('/cmd_vel',{}).get('publishers')==['collision_monitor'],
            map_odom_only_slam=s['map_odom_authorities']==['slam_toolbox'] and len(s['map_odom_gids'])==1,
            no_initialpose=not self.node.initializations,
            physical_obstacle_count=manifest['model_dimensions']['nmocap']==(0 if self.args.static else 1),
            raw_scan_only_simulator=graph.get('/scan',{}).get('publishers')==['t1_sim'],
            safety_reads_raw_scan='collision_monitor' in graph.get('/scan',{}).get('subscribers',[]),
            nav2_reads_raw_scan=all(name in graph.get('/scan',{}).get('subscribers',[])
                                    for name in ('local_costmap','global_costmap')),
            mapping_filter_only=graph.get('/scan/mapping',{}).get('publishers')==['t1_mapping_scan_filter'],
            slam_reads_filtered_scan=graph.get('/scan/mapping',{}).get('subscribers')==['slam_toolbox'])
        s['task_consistent'] = manifest.get('exploration_task')==self.args.task_data==result.get('task')
        s['headless'] = self.args.headless
        if not self.args.headless:
            expected = dict(anchor_odom=result.get('anchor_odom'), goal_odom=result.get('goal_odom'))
            # JSON normalizes the immutable display tuples for comparison.
            s['display_coordinates_consistent'] = (manifest.get('exploration_display') == expected
                == json.loads(json.dumps(s.get('display_start'))))
        exits = (all(v==0 for v in s.get('parent_exit_codes',{}).values())
                 and bool(s.get('launch_process_exit_codes'))
                 and all(v['exit_code']==0 for v in s['launch_process_exit_codes'].values()))
        s['infrastructure_valid'] = bool(s['simulator_exit_code']==0 and not s['fell']
            and s['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0)
            and all(s['graph_contract'].values()) and s['task_consistent']
            and s.get('safe_stop',{}).get('physical_stationary')
            and not s.get('remaining_owned_pids') and not s.get('remaining_project_processes')
            and s.get('clock_regressions')==0 and s.get('slam_shutdown',{}).get('complete')
            and not s.get('cleanup_error'))
        n = self.node
        s['static_control'] = self.args.static
        if not self.args.static:
            from advanced.verification import blocked_wait_evidence, response_verdict
            active = [o for o in n.obstacles if o['phase'] not in ('staged','finished')]
            positions = [o['position'][:2] for o in n.obstacles]
            s['dynamic'] = dict(triggered=bool(active), finished=bool(n.obstacle and n.obstacle['phase']=='finished'),
                actual_travel_m=sum(math.dist(a,b) for a,b in zip(positions,positions[1:])),
                active_scan_hit_frames=sum(bool(x['hits']) and x['obstacle']['phase'] not in ('staged','finished') for x in n.dynamic_scans),
                active_costmap_marked_frames=sum(x['max_cost']==100 and x['obstacle']['phase'] not in ('staged','finished') for x in n.cost_samples),
                blocked_wait=blocked_wait_evidence(n.rows,n.obstacles,goal or [math.inf,math.inf]))
            s['dynamic'].update(response_verdict(s['dynamic'],s['arrived']))
        dynamic_ok = self.args.static or (s['dynamic']['verdict']=='PASS' and s['dynamic']['actual_travel_m']>=1.)
        graphics_ok = self.args.headless or (s['display_coordinates_consistent'] and not s.get('gpu_monitor_error')
            and bool(s.get('arrived_screenshots')))
        s['validation_before_port_check'] = bool(s['infrastructure_valid'] and s['arrived'] and exits and dynamic_ok
                                               and graphics_ok and not s.get('error'))
        base.save('observation.json', dict(rows=n.rows, plans=n.plans, obstacles=n.obstacles,
                  cost_samples=n.cost_samples, dynamic_scans=n.dynamic_scans))
        s['status'] = 'PENDING_PORT_CHECK' if s['validation_before_port_check'] else 'FAILED'

    def save_map(self, label):
        files = super().save_map(label)
        # Advanced's historical default binds its own fixed scene. W16 also
        # accepts alternate simulator worlds, so bind the actual input here.
        scene = Path(self.args.scene)
        if not scene.is_absolute():
            scene = WS/'src/t1_mujoco/worlds'/f'{self.args.scene}.yaml'
        meta = OUT/'maps'/f'{label}.meta.json'
        data = json.loads(meta.read_text())
        data.update(scene_file=str(scene),scene_sha256=base.sha(scene),
                    slam_file=str(OUT/'slam.yaml'),slam_sha256=base.sha(OUT/'slam.yaml'))
        meta.write_text(json.dumps(data,indent=2)+'\n')
        return files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', type=Path, default=WS/'src/t1_navigation/config/exploration_task.yaml')
    parser.add_argument('--scene', default='advanced_world', help='Simulator-only scene, never passed to explorer')
    parser.add_argument('--static', action='store_true', help='Explicit control run without a dynamic body')
    parser.add_argument('--time-limit', type=float, default=600.)
    parser.add_argument('--preflight', action='store_true')
    parser.add_argument('--headless', action='store_true', help='CPU-only run without windows')
    parser.add_argument('--no-rviz', action='store_true', help='Show only the MuJoCo window')
    parser.add_argument('--hold', action='store_true', help='Keep windows open after safe arrival until Ctrl+C')
    args = parser.parse_args()
    args.startup_check, args.gpu_watch = False, None
    if not 0 < args.time_limit <= 800:
        parser.error('time-limit must be in (0, 800] simulation seconds')
    if not args.static and args.scene != 'advanced_world':
        parser.error('EXP01 requires advanced_world; use --static for other layouts')
    (WS/'run_data/demo').mkdir(parents=True, exist_ok=True)
    lock = (WS/'run_data/demo/active.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    signal.signal(signal.SIGINT, lambda *_: setattr(base,'STOP',True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(base,'STOP',True))
    if args.headless or args.preflight:
        from cpu_guard import enter
        base.save('cpu-guard.json', enter())
    from t1_mujoco.exploration_task import load_task, SNAPSHOT_ENV
    args.task_data = load_task(args.task)
    (OUT/'task.yaml').write_bytes(args.task.read_bytes())
    assert load_task(OUT/'task.yaml') == args.task_data
    os.environ[SNAPSHOT_ENV] = str(OUT/'task.yaml')
    import yaml
    slam = yaml.safe_load((WS/'src/t1_navigation/config/slam.yaml').read_text())
    slam['slam_toolbox']['ros__parameters']['scan_topic'] = '/scan/mapping'
    (OUT/'slam.yaml').write_text(yaml.safe_dump(slam))
    simulation = yaml.safe_load((WS/'src/t1_navigation/config/w12_mapping_runtime.yaml').read_text())
    # Display refresh only: preserve physics, policy, LiDAR and safety timing.
    simulation['t1_sim']['ros__parameters']['viewer_rate_hz'] = 15.
    (OUT/'simulation.yaml').write_text(yaml.safe_dump(simulation))
    provenance = advanced.preflight()
    provenance.update(task=args.task_data, simulator_scene=args.scene, static_control=args.static,
                      mapping_params_sha256=base.sha(OUT/'slam.yaml'),
                      simulation_params_sha256=base.sha(OUT/'simulation.yaml'))
    base.save('provenance.json', provenance)
    if args.preflight:
        return 0
    if not args.headless:
        from gpu_watch import Watch, GPU_ENV
        os.environ.update(GPU_ENV)
        args.gpu_watch = Watch(OUT/'gpu', idle_wait_s=60, idle_max_utilization_pct=60.)
    run = ExplorationRun(args, provenance)
    base.say(f'W16 run {RUN_ID}; {OUT}')
    try:
        run.execute()
    except Exception as exc:
        run.summary['error'] = repr(exc)
        (OUT/'failure.txt').write_text(traceback.format_exc())
        base.say(f'FAILED: {exc}')
    finally:
        try:
            if run.node:
                run.runtime_graph = run.node.graph(('/cmd_vel','/map','/tf','/t1/eval/ground_truth','/t1/eval/obstacle'))
            run.cleanup()
        except Exception:
            run.summary['cleanup_error'] = traceback.format_exc()
        try:
            run.metrics()
        except Exception:
            run.summary.update(metric_error=traceback.format_exc(), status='FAILED')
        base.save('result.json', run.summary)
    return 0 if run.summary.get('validation_before_port_check') else 1


if __name__ == '__main__':
    raise SystemExit(main())

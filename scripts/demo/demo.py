"""W14 packaging only. Reuses frozen sensors, goal tool and offline predicates."""
import argparse
from datetime import datetime, timezone
import fcntl
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
import traceback
import threading

WS = Path(__file__).resolve().parents[2]
OUT = Path(os.environ['T1_DEMO_DIR'])
RUN_ID = os.environ['T1_DEMO_RUN_ID']
BASELINE = 'b23a92465b317ca781d3eb210051126c5786908e'
STOP = False
GPU_PROFILE=WS/'scripts/demo/gpu-profile.json'
SAFETY_BLOCK_REASON = 'W14_BLOCKED: staged NVIDIA validation has not authorized this GUI mode.'


def say(message):
    print(f'[demo] {message}', flush=True)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def command(args, **kwargs):
    return subprocess.check_output(args, text=True, timeout=20, **kwargs).strip()


def git(*args):
    return command(['git', '-C', str(WS), *args])


def save(name, data):
    target = OUT/name
    temp = target.with_suffix(target.suffix+'.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, default=str)+'\n')
    temp.replace(target)


def identity(pid):
    try:
        stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=pid, ppid=int(stat[1]), start=stat[19], state=stat[0],
                    command=Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace'))
    except (OSError, IndexError, ValueError):
        return None


def processes():
    return {int(p.name): r for p in Path('/proc').iterdir()
            if p.name.isdigit() and (r := identity(int(p.name)))}


def alive(record):
    actual = identity(record['pid'])
    return actual and actual['start'] == record['start'] and actual['state'] != 'Z'


def heartbeat_progress(previous, token, now):
    """Watch file changes with monotonic elapsed time; wall-clock steps are irrelevant."""
    if previous is None or previous[0] != token:
        return (token, now), False
    return previous, now-previous[1] > 3.


def clock_stalled(node,now,last_progress):
    # The simulator publishes its initial timestamp before taking its first
    # physics step. That single sample does not establish a running clock.
    return (node is not None and node.first_clock is not None
            and node.sim>node.first_clock and now-last_progress>3)


def project_processes():
    result = []
    for r in processes().values():
        parts = r['command'].split()
        if not parts:
            continue
        executable = Path(parts[0]).name
        if (executable in ('t1_sim', 'navigation_starter', 'navigate_to_pose', 'command_gate', 'navigation_adapter','t1_demo_rviz')
            or ('ros2' in executable and 'launch t1_bringup' in r['command'])
            or (executable == 'rviz2' and str(WS) in r['command'])):
            result.append(r)
    return result


def port_free():
    try:
        with socket.socket() as s:
            s.bind(('127.0.0.1', 42127))
        return True
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            return False
        raise RuntimeError(f'Cannot inspect DDS port: {exc}; run in a normal Ubuntu terminal with local sockets available.') from exc


def port_connections():
    records=[]
    for family in ('tcp','tcp6'):
        for line in (Path('/proc/net')/family).read_text().splitlines()[1:]:
            fields=line.split()
            if int(fields[1].split(':')[1],16)==42127:
                records.append(dict(family=family,local=fields[1],state=fields[3]))
    return dict(listeners=[r for r in records if r['state']=='0A'],
                time_wait=[r for r in records if r['state']=='06'],
                other_connections=[r for r in records if r['state'] not in ('0A','06')])


def wait_port_reusable(timeout=65.):
    """TCP TIME_WAIT is kernel state, never a reason to kill another process."""
    end=time.monotonic()+timeout
    announced=False
    while not port_free():
        state=port_connections()
        if state['listeners'] or state['other_connections'] or time.monotonic()>=end:
            return False
        if not announced:
            say('DDS exited; waiting up to 65s for kernel TCP TIME_WAIT to expire.')
            announced=True
        time.sleep(.1)
    return True


def preflight(args):
    import yaml
    import mujoco
    import torch
    import rclpy
    from ament_index_python.packages import get_package_prefix
    info = dict(line.strip().split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
    assert info['ID'].strip('"') == 'ubuntu' and info['VERSION_ID'].strip('"') == '24.04', 'Use Ubuntu-24.04.'
    assert os.environ.get('ROS_DISTRO') == 'jazzy', 'ROS_DISTRO must be jazzy; check scripts/env.sh.'
    assert Path(sys.prefix).resolve() == Path(os.environ['T1_VENV']).resolve(), 'Wrong interpreter; check T1_VENV.'
    assert sys.version_info[:2] == (3, 12) and mujoco.__version__ == '3.15.0', 'Restore frozen Python 3.12 / MuJoCo 3.15.0 (docs/baseline); do not upgrade.'
    for name in ('ros2', 'fast-discovery-server'):
        assert shutil.which(name), f'Missing {name}; source ROS Jazzy and check docs/baseline/apt-packages.txt.'
    lock = json.loads((WS/'scripts/demo/baseline-lock.json').read_text())
    presentation_lock = json.loads((WS/'scripts/demo/presentation-lock.json').read_text())
    lock.update(presentation_lock)
    changed = [p for p, h in lock.items() if not (WS/p).is_file() or sha(WS/p) != h]
    assert not changed, f'Frozen baseline changed: {changed}; inspect git diff, do not overwrite edits.'
    exclusions = [f':(exclude){p}' for p in presentation_lock]
    assert not git('diff', BASELINE, '--', 'src', *exclusions), 'Unexpected source change relative to formal-v1.'
    up, assets = Path(os.environ['T1_UPSTREAM']), Path(os.environ['T1_ASSETS'])
    for root, expected in ((up, '7bb1462e2ac742eba5c01659b815e2e99fbe617f'),
                           (assets, '12a97516f57469078707afb587a5f84c1cb110d1')):
        assert command(['git', '-C', str(root), 'rev-parse', 'HEAD']) == expected, f'Wrong frozen dependency SHA: {root}'
        assert not command(['git', '-C', str(root), 'status', '--porcelain']), f'Dependency has edits: {root}; preserve and inspect.'
    checkpoint = up/'tasks/locomotion/robots/t1/models/t1_walk.pt'
    model = assets/'robots/T1/T1_23dof.xml'
    assert sha(checkpoint) == 'ef1d61e19082b83405f4320a08f4cfc2d7d7f003ed3790dab013778ba442dec7', 'Wrong T1 checkpoint; see docs/baseline/assets-manifest.md.'
    assert sha(model) == 'f00e3410362b1e2267e6db05d25c33d6240bbf2436d62a409f07042bad4f7fb2', 'Wrong T1 MJCF.'
    m = mujoco.MjModel.from_xml_path(str(model))
    assert (m.nq, m.nv, m.nu) == (30, 29, 23), 'Incompatible model dimensions.'
    for package in ('t1_mujoco', 't1_bringup', 't1_description', 't1_navigation'):
        assert Path(get_package_prefix(package)).is_relative_to(WS/'install'), f'{package} overlay is not this workspace; rebuild via /usr/bin/colcon.'
    obsolete=WS/'install/t1_navigation/share/ament_index/resource_index/nav2_core__pluginlib__plugin/t1_navigation'
    assert not obsolete.exists(), f'Stale experimental plugin index: {obsolete}; archive generated artifacts and rebuild the formal-v1 overlay, do not alter source.'
    import t1_bringup.navigation_tools as tools
    assert Path(tools.__file__).resolve() == WS/'src/t1_bringup/t1_bringup/navigation_tools.py', 'Stale overlay; use colcon build --symlink-install.'
    wrapper = WS/'install/t1_mujoco/lib/t1_mujoco/t1_sim'
    assert 'T1_VENV' in wrapper.read_text(), 'Simulation wrapper must explicitly execute configured venv.'
    map_path = WS/'src/t1_navigation/maps/dynamic_crossing_world.yaml'
    map_spec = yaml.safe_load(map_path.read_text())
    assert (map_path.parent/map_spec['image']).is_file(), 'Map image missing; restore frozen map, do not regenerate.'
    leftovers = project_processes()
    assert not leftovers, f'Existing project processes: {[r["pid"] for r in leftovers]}; stop their owning terminal, never pkill.'
    assert port_free(), f'DDS TCP42127 not bindable: {port_connections()}; if only TIME_WAIT, wait for kernel expiry. Otherwise inspect the known owner; never pkill.'
    graphics = dict(requested=not (args.headless or args.preflight), cuda_probe=False, cpu_guard=args.cpu_guard)
    if not (args.headless or args.preflight):
        from gpu_watch import verify_gl
        verify_gl(args.gpu_watch,OUT)
        graphics.update(status='NVIDIA4050_VERIFIED',rviz='NVIDIA lifecycle wrapper')
    else:
        graphics['status']='CPU_ONLY'
    data = dict(run_id=RUN_ID, code_sha=git('rev-parse', 'HEAD'), dirty=git('status', '--porcelain'),
                baseline=BASELINE, scenario='U01', seed=4000, start=[0,0,0], goal=[9,0,0],
                interpreter=sys.executable, python=sys.version, mujoco=mujoco.__version__, torch=torch.__version__,
                ros=os.environ['ROS_DISTRO'], domain=37, dds_port=42127, graphics=graphics,
                locked_files=len(lock), resource_hashes={str(checkpoint):sha(checkpoint), str(model):sha(model)})
    save('preflight.json', data)
    say(f'Preflight PASS | U01 seed 4000 | code {data["code_sha"][:12]} | DDS42127/domain37')
    return data


class Demo:
    managed_shutdown_order = ('goal', 'rviz', 'launch', 'dds')
    def __init__(self, args, provenance):
        self.args, self.provenance = args, provenance
        self.started = time.monotonic()
        self.children, self.owned = {}, {}
        self.node = None
        self.cleanup_started = False
        self.summary = dict(run_id=RUN_ID, code_sha=provenance['code_sha'], scenario='U01', seed=4000,
                            started_utc=datetime.now(timezone.utc).isoformat(), arrived=False,
                            human_intervention=False, status='STARTING')
        self.last_status = 0
        self.last_clock_seen = None
        self.clock_changed_wall = self.started
        self.trace_progress = None
        self.gpu_watch=getattr(args,'gpu_watch',None)
        self.gpu_error=None
        self.gpu_thread_stop=threading.Event()
        self.gpu_thread=None
        self.log = (OUT/'runtime.log').open('a')
        # ROS diagnostics go to a file, progress stays on stdout.
        os.dup2(self.log.fileno(), 2)
        self.launch_log = OUT/'launch.log'
        self.goal_log = OUT/'goal.log'

    def start_gpu_watch(self):
        if self.gpu_watch is None:return
        def monitor():
            while not self.gpu_thread_stop.wait(.1):
                try:
                    self.gpu_watch.check([self.launch_log,OUT/'rviz.log',OUT/'runtime.log'])
                except Exception as exc:
                    self.gpu_error=repr(exc);self.gpu_watch.halt(exc)
                    # Newest renderer first; only PID/starttime-registered children.
                    for item in list(self.owned.values()):
                        if item['owner']=='rviz' and alive(item):
                            try:os.kill(item['pid'],signal.SIGTERM)
                            except ProcessLookupError:pass
                    for item in list(self.owned.values()):
                        if 't1_mujoco.simulation' in item['command'] and alive(item):
                            try:os.kill(item['pid'],signal.SIGTERM)
                            except ProcessLookupError:pass
                    break
        self.gpu_thread=threading.Thread(target=monitor,name='demo_gpu_watch',daemon=True)
        self.gpu_thread.start()

    def stop_gpu_watch(self):
        if not hasattr(self,'gpu_thread_stop'):return
        self.gpu_thread_stop.set()
        if self.gpu_thread:self.gpu_thread.join(timeout=5)
        if self.gpu_watch:
            if not self.gpu_watch.lock.closed:
                try:self.gpu_watch.host.check(__import__('gpu_watch').ERROR,windows=True)
                except Exception as exc:
                    self.gpu_error=repr(exc);self.gpu_watch.halt(exc)
            self.summary['gpu_monitor_error']=self.gpu_error
            self.summary['gpu_samples']=self.gpu_watch.samples
            self.gpu_watch.close()

    def spawn(self, name, args):
        log = (OUT/f'{name}.log').open('w')
        p = subprocess.Popen(args, cwd=WS, stdin=subprocess.DEVNULL, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
        self.children[name] = (p, log)
        self.track()
        return p

    def track(self):
        records = processes()
        pending = [(p.pid,name) for name,(p, _) in self.children.items()]
        while pending:
            pid,owner = pending.pop()
            if pid in records:
                self.owned[pid] = dict(records[pid],owner=owner)
                pending += [(r['pid'],owner) for r in records.values() if r['ppid'] == pid]
        save('owned-processes.json', list(self.owned.values()))

    def spin(self, check_stop=True):
        import rclpy
        if check_stop and self.gpu_error:
            raise RuntimeError(f'GPU monitoring stopped this run: {self.gpu_error}')
        if check_stop and STOP:
            raise InterruptedError('User requested stop')
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=.01)
        now=time.monotonic()
        if check_stop and 'rviz' in self.children:
            heartbeat=OUT/'rviz-heartbeat'
            if heartbeat.exists():
                self.rviz_heartbeat_progress, stalled = heartbeat_progress(
                    getattr(self, 'rviz_heartbeat_progress', None), heartbeat.stat().st_mtime_ns, now)
                if stalled:
                    raise RuntimeError('RViz Qt event loop unresponsive for3s; no retry')
        if self.node and self.node.sim != self.last_clock_seen:
            self.last_clock_seen=self.node.sim
            self.clock_changed_wall=now
        # Receiving no first sample is startup/discovery, not a stopped clock.
        # Independently watch the simulator's flushed trace so DDS absence cannot
        # mask a physical/viewer stall. The established-clock deadline stays 3s.
        trace=WS/'run_data'/RUN_ID/'sim_trace.csv'
        if check_stop and trace.exists():
            size=trace.stat().st_size
            if self.trace_progress is None or self.trace_progress[0]!=size:
                self.trace_progress=(size,now)
            if now-self.trace_progress[1]>(3 if size>4096 else 15):
                raise RuntimeError('Simulator trace stopped advancing; stopping this run')
        if check_stop and clock_stalled(self.node,now,self.clock_changed_wall):
            raise RuntimeError('Simulation clock stalled for 3 wall seconds; stopping this run')
        if time.monotonic()-self.last_status >= 1:
            self.track()
            self.last_status = time.monotonic()
            for name in ('dds','launch','rviz'):
                if name in self.children and self.children[name][0].poll() is not None:
                    raise RuntimeError(f'{name} exited unexpectedly: {self.children[name][0].returncode}')
            if self.node and self.node.state and self.node.state['state'] in ('FALLEN','FAULT'):
                raise RuntimeError(f'Simulation state {self.node.state}')
            # launch captures every managed node's exit; zero exits of one-shot tools are expected.
            text = self.launch_log.read_text(errors='replace') if self.launch_log.exists() else ''
            if 'process has died' in text:
                raise RuntimeError('A launch component died; see launch.log')
        if self.node and time.monotonic()-getattr(self, 'printed', 0) > 5:
            self.printed = time.monotonic()
            n = self.node
            say(f'{self.summary["status"]} | sim {n.sim:.1f}s | gate {(n.gate or {}).get("state", "waiting")} | CM {n.cm} | obstacle {(n.obstacle or {}).get("phase", "waiting")} | plans {len(n.plans)}')
            self.markers()

    def until(self, predicate, timeout=30., check_stop=True):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() > end:
                raise TimeoutError(f'{self.summary["status"]}: readiness deadline ({timeout}s)')
            self.spin(check_stop)

    def call(self, kind, name, request, timeout=5., check_stop=True):
        client = self.node.create_client(kind, name)
        try:
            self.until(client.service_is_ready, timeout, check_stop)
            future = client.call_async(request)
            self.until(future.done, timeout, check_stop)
            response = future.result()
            if response is None:
                raise RuntimeError(f'{name}: empty response')
            return response
        finally:
            self.node.destroy_client(client)

    def markers(self):
        if not hasattr(self, 'viz'):
            return
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        n = self.node
        a = MarkerArray()
        for i, (x, text, color) in enumerate(((0.,'A  START',(.05,.55,.8)),(9.,'B  GOAL',(.05,.6,.25)))):
            m=Marker(); m.header.frame_id='map'; m.ns='route'; m.id=2*i
            m.type=Marker.LINE_STRIP; m.action=Marker.ADD; m.pose.orientation.w=1.; m.scale.x=.045
            m.color.r,m.color.g,m.color.b=color; m.color.a=1.
            m.points=[Point(x=x+.65*math.cos(t*math.pi/40),y=.65*math.sin(t*math.pi/40),z=.06) for t in range(81)]
            a.markers.append(m)
            label=Marker(); label.header.frame_id='map'; label.ns='route'; label.id=2*i+1
            label.type=Marker.TEXT_VIEW_FACING; label.action=Marker.ADD; label.pose.orientation.w=1.
            label.pose.position.x=x; label.pose.position.y=-1.05; label.pose.position.z=.15
            label.scale.z=.42; label.color.r,label.color.g,label.color.b=color; label.color.a=1.;label.text=text
            a.markers.append(label)
        self.viz.publish(a)

    def screenshot(self, stage):
        if self.args.headless:
            return
        tree = command(['xwininfo', '-root', '-tree'])
        wanted = ['MuJoCo'] + ([] if self.args.no_rviz else ['RViz'])
        images=[]
        for title in wanted:
            windows=[line.split()[0] for line in tree.splitlines() if '"' in line and title in line]
            if not windows:
                raise RuntimeError(f'{title} window not visible')
            path=OUT/f'{stage}-{title.lower()}.png'
            subprocess.run(['import','-window',windows[-1],str(path)],check=True,timeout=15)
            images.append(str(path))
        self.summary[f'{stage}_screenshots']=images

    def execute(self):
        os.environ['T1_EVAL_OUT']=str(OUT)
        os.environ['T1_DEMO_PRESENTATION']='1'
        sys.path.insert(0,str(WS/'scripts'))
        from evaluation.harness import Probe
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid
        from lifecycle_msgs.srv import GetState
        from rclpy.time import Time
        from visualization_msgs.msg import MarkerArray
        import rclpy
        self.start_gpu_watch()
        self.spawn('dds',[str(WS/'scripts/start_local_dds.sh')])
        # A participant created before the server listens can miss discovery on
        # this Fast DDS/WSL setup. Create no ROS participant until real LISTEN.
        self.until(lambda:'### Server is running ###' in (OUT/'dds.log').read_text(errors='replace')
                   and bool(port_connections()['listeners']),10)
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node=Probe()
        from grid_visuals import GridVisuals
        self.grid_visuals=GridVisuals(self.node)
        self.viz=self.node.create_publisher(MarkerArray,'/t1/viz/demo',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.global_map=None
        self.node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:setattr(self,'global_map',m),
                                      QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.action=ActionClient(self.node,NavigateToPose,'navigate_to_pose')
        self.spawn('launch',['ros2','launch','t1_bringup','navigation.launch.py',
            'scene:=dynamic_crossing_world','start_pose:=0 0 0','initial_pose:=0 0 0',
            f'headless:={str(self.args.headless).lower()}','rviz:=false',
            f'rviz_config:={WS}/src/t1_navigation/config/demo.rviz',f'run_id:={RUN_ID}',
            'seed:=4000','target_rtf:=1.0','max_sim_time:=0',
            f'map:={WS}/src/t1_navigation/maps/dynamic_crossing_world.yaml',
            f'localization_params_file:={WS}/src/t1_navigation/config/w12_localization.yaml',
            f'events_file:={WS}/src/t1_bringup/config/w13_events_v1.yaml','scenario:=U01'])
        self.summary['status']='WAITING_FOR_READINESS'
        self.until(lambda:'"starter": "navigation active"' in self.launch_log.read_text(errors='replace'),120)
        n=self.node
        self.until(lambda:n.state and n.state['state']=='READY' and n.first_clock is not None and n.sim>n.first_clock+.5
                   and n.scan and -.05<=n.sim-(n.scan.header.stamp.sec+n.scan.header.stamp.nanosec*1e-9)<=.3
                   and n.map and n.pose and n.map_pose() and n.cost_samples and self.global_map
                   and len(n.initializations)==1 and n.gate and not n.gate.get('health_reason')
                   and self.action.server_is_ready(),30)
        for parent,child in (('map','odom'),('odom','base_footprint'),('base_footprint','laser_stabilized')):
            assert n.buffer.can_transform(parent,child,Time()), f'Missing TF {parent}->{child}'
        lifecycle={}
        for name in ('map_server','amcl','controller_server','planner_server','behavior_server','bt_navigator',
                     'global_costmap/global_costmap','local_costmap/local_costmap','velocity_smoother','collision_monitor'):
            lifecycle[name]=self.call(GetState,f'/{name}/get_state',GetState.Request()).current_state.id
        assert all(v==3 for v in lifecycle.values()), f'Inactive nodes: {lifecycle}'
        self.summary.update(readiness_wall_s=time.monotonic()-self.started, lifecycle=lifecycle, initializations=n.initializations)
        from evaluation.episode import check_parameters
        params, checks=check_parameters(self)
        self.summary.update(parameters=params, parameter_checks=checks)
        assert all(checks.values()), f'Parameter contract failed: {checks}'
        if not self.args.headless and not self.args.no_rviz:
            os.environ['T1_RVIZ_HEARTBEAT']=str(OUT/'rviz-heartbeat')
            self.spawn('rviz',[str(WS/'build/demo-rviz/t1_demo_rviz'),'-d',str(WS/'src/t1_navigation/config/demo.rviz'),
                              '--ros-args','-p','use_sim_time:=true'])
            self.until(lambda:(OUT/'rviz-heartbeat').exists() and 'RVIZ_LIFECYCLE ready' in (OUT/'rviz.log').read_text(errors='replace'),25)
            from gpu_watch import renderer_verified
            assert renderer_verified((OUT/'rviz.log').read_text()),'RViz actual renderer is not NVIDIA4050'
            drawing_until=n.sim+2.;self.until(lambda:n.sim>=drawing_until,10)
        self.summary['readiness_wall_s']=time.monotonic()-self.started
        self.screenshot('ready')
        if self.args.startup_check:
            self.summary['status']='STANDING_GPU_CHECK'
            end=n.sim+30.;self.until(lambda:n.sim>=end,50)
            self.safe_stop();self.summary['startup_check_completed']=True
            return
        say('Ready. Sending A -> B automatically. Explain scan, costmaps and W10 stop/resume.')
        self.summary['status']='NAVIGATING'
        goal=self.spawn('goal',['ros2','run','t1_bringup','navigate_to_pose','9','0','0','--time-limit','120'])
        self.until(lambda:goal.poll() is not None,320)
        lines=[json.loads(l) for l in self.goal_log.read_text().splitlines() if l.startswith('{')]
        nav=next((l for l in reversed(lines) if l.get('event')=='result'),{})
        self.summary['nav2']=nav
        if goal.returncode!=0 or nav.get('status')!='SUCCEEDED' or not nav.get('verified'):
            raise RuntimeError(f'Navigation failed: exit={goal.returncode}, status={nav.get("status")} (no seed retry)')
        post=n.sim+12.
        self.until(lambda:n.sim>=post,40)
        self.summary['task_wall_s']=time.monotonic()-self.started
        self.screenshot('arrived')
        # Stop before holding the final view, while all services are still alive.
        self.safe_stop()
        self.summary['status']='ARRIVAL_PENDING_OFFLINE_CHECK'
        save('partial-result.json',self.summary)
        say('Nav2 verified arrival; safety stop complete. Independent metrics follow cleanup.')
        if not self.args.headless:
            self.summary['status']='FINAL_VIEW'
            end=time.monotonic()+8.
            say('Final view held; Ctrl+C to finish.' if self.args.hold else 'Final view for 8s, then automatic cleanup.')
            while not STOP and (self.args.hold or time.monotonic()<end):
                self.spin(False)

    def safe_stop(self):
        from std_srvs.srv import Trigger
        from evaluation.harness import angle
        try:
            response=self.call(Trigger,'/t1/safety/stop',Trigger.Request(),3.,False)
            assert response.success,response.message
            n=self.node
            def stopped():
                if not n.gate or n.gate['state']!='STOPPED' or len(n.rows)<27:
                    return False
                rows=n.rows[-27:]; held=rows[-21:]
                if held[-1]['sim']-held[0]['sim']<2.0-1e-6:
                    return False
                for row in held:
                    if not row['u_pol'] or any(abs(v)>1e-9 for v in row['u_pol']):
                        return False
                    prior=[r for r in rows if r['sim']<=row['sim']-.5+1e-6]
                    if not prior:
                        return False
                    old=prior[-1];dt=row['sim']-old['sim']
                    if not .49<=dt<=.7 or math.hypot(row['x']-old['x'],row['y']-old['y'])/dt>=.05 or abs(angle(row['yaw']-old['yaw']))/dt>=.05:
                        return False
                return True
            self.until(stopped,10.,False)
            self.summary['safe_stop']=dict(ack=True,physical_stationary=True,gate=n.gate)
        except Exception as exc:
            self.summary['safe_stop']=dict(ack=False,error=repr(exc))
            say('Safety stop acknowledgement/standstill unavailable; cleanup proceeds, watchdog remains active.')

    def cleanup(self):
        if self.cleanup_started:
            return
        self.cleanup_started=True
        self.summary['shutdown_signals']=[]
        if self.node and 'launch' in self.children and self.children['launch'][0].poll() is None:
            self.safe_stop()
        self.track()
        # One SIGINT to launch parent; launch forwards once to its children.
        for name in self.managed_shutdown_order:
            if name not in self.children:
                continue
            p,log=self.children[name]
            if name=='dds':
                # Observation only: do not change frozen navigation or signal order.
                self.summary['dds_shutdown']=dict(pid=p.pid,alive_before_signal=p.poll() is None,
                    ros_communication_healthy=not self.summary.get('error') and bool(self.summary.get('nav2',{}).get('verified')),
                    navigation_and_stop_complete=bool(self.summary.get('nav2',{}).get('verified')) and
                        self.summary.get('safe_stop',{}).get('ack') is True and
                        self.summary.get('safe_stop',{}).get('physical_stationary') is True,
                    signal=None)
                if getattr(self, 'close_observer_before_dds', False):
                    try:
                        self.close_observer()
                        deadline=time.monotonic()+5.
                        connections=port_connections()
                        while connections['other_connections'] and time.monotonic()<deadline:
                            time.sleep(.05)
                            connections=port_connections()
                        self.summary['dds_shutdown'].update(
                            observer_closed_before_signal=True,
                            connections_after_observer_close=connections,
                            clients_disconnected=not connections['other_connections'])
                        if connections['other_connections']:
                            raise RuntimeError('DDS clients still connected after observer shutdown')
                    except Exception as exc:
                        # Continue owned-process cleanup, but never pass this run.
                        self.summary['observer_shutdown_error']=repr(exc)
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                self.summary['shutdown_signals'].append([name,p.pid,'SIGINT'])
                if name=='dds':self.summary['dds_shutdown']['signal']='SIGINT'
            try:
                p.wait(timeout=35 if name=='launch' else 10)
            except subprocess.TimeoutExpired:
                # Only individually registered PIDs, with starttime guards; no process-group wildcard.
                self.track()
                for sig,wait in ((signal.SIGTERM,5),(signal.SIGKILL,3)):
                    candidates=[r for r in self.owned.values() if alive(r) and r['owner']==name]
                    for r in candidates:
                        os.kill(r['pid'],sig)
                        self.summary['shutdown_signals'].append([name,r['pid'],sig.name])
                    try:
                        p.wait(timeout=wait); break
                    except subprocess.TimeoutExpired:
                        pass
            log.close()
        # Reap orphaned descendants after launcher failure, limited to registered identities.
        remaining=[r for r in self.owned.values() if alive(r)]
        for sig in (signal.SIGTERM,signal.SIGKILL):
            for r in remaining:
                if alive(r):
                    os.kill(r['pid'],sig)
                    self.summary['shutdown_signals'].append(['orphan',r['pid'],sig.name])
            deadline=time.monotonic()+3
            while any(alive(r) for r in remaining) and time.monotonic()<deadline:
                time.sleep(.05)
        self.stop_gpu_watch()
        if any(row[-1]=='SIGKILL' for row in self.summary['shutdown_signals']):
            self.summary['error']='SIGKILL was needed; mandatory stop, no further tests'
        self.summary['parent_exit_codes']={name:p.poll() for name,(p,_) in self.children.items()}
        self.summary['dds_returncode']=self.summary['parent_exit_codes'].get('dds')
        if getattr(self,'gpu_watch',None) and (self.summary['parent_exit_codes'].get('rviz',0)!=0 or any(row[-1]=='SIGKILL' for row in self.summary['shutdown_signals'])):
            self.gpu_watch.halt('Abnormal RViz exit or forced kill; strict authorization requires stop, no retry')
        text=self.launch_log.read_text(errors='replace') if self.launch_log.exists() else ''
        starts=dict(re.findall(r'\[INFO\] \[([^]]+)\]: process started with pid \[(\d+)\]',text))
        exits={name:0 for name in re.findall(r'\[INFO\] \[([^]]+)\]: process has finished cleanly',text)}
        for name,code in re.findall(r'\[ERROR\] \[([^]]+)\]: process has died.*?exit code (-?\d+)',text):
            exits[name]=int(code)
        self.summary['launch_process_exit_codes']={name:dict(pid=int(pid),exit_code=exits.get(name)) for name,pid in starts.items()}
        self.summary['remaining_owned_pids']=[r['pid'] for r in self.owned.values() if alive(r)]
        self.summary['remaining_project_processes']=project_processes()
        self.close_observer()
        self.summary['dds_port_state_before_runner_exit']=port_connections()
        self.summary['dds_port_released']=None
        self.summary['dds_port_check_deferred']=True
        self.summary['wall_total_s']=time.monotonic()-self.started

    def close_observer(self):
        """Release this process's ROS participant once, retaining recorded evidence."""
        if self.node and not getattr(self, '_observer_closed', False):
            self.summary['events']=self.node.events
            self.summary['graph']=self.runtime_graph
            self.summary['initializations']=self.node.initializations
            self.summary['clock_regressions']=self.node.clock_regressions
            self.action.destroy() if hasattr(self,'action') else None
            self.node.destroy_node()
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
            self._observer_closed=True

    def metrics(self):
        sys.path.insert(0,str(WS/'scripts'))
        from evaluation.episode import load_trace,settle_analysis
        from evaluation.metrics import body_clearance
        artifact=WS/'run_data'/RUN_ID
        manifest=json.loads((artifact/'manifest.json').read_text())
        trace=load_trace(RUN_ID)
        nav=self.summary.get('nav2',{})
        success_sim=nav.get('nav2_success_sim')
        settle=settle_analysis(trace,(9.,0.,0.),success_sim or trace['sim_time'][-1],
                               dict(position_m=.3,yaw_deg=20),2.)
        self.summary.update(settle=settle, contacts=manifest.get('contact_counts'),
            fell=any(s in ('FALLEN','FAULT') for s in trace['state']), simulator_exit_code=manifest.get('exit_code'),
            body_clearance=body_clearance(manifest,artifact/'geometry.jsonl'))
        n=self.node
        hit_frames=sum(bool(s['hits']) and s['obstacle']['triggered_at'] is not None for s in n.dynamic_scans)
        marked=sum(c['max_cost']==100 and c['obstacle']['triggered_at'] is not None for c in n.cost_samples)
        triggered=next((o['triggered_at'] for o in n.obstacles if o['triggered_at'] is not None),None)
        # Frozen W12 evidence predicates; output under this unique ignored run only.
        from evaluation import episode as ep
        os.environ['T1_EVAL_KIND']='approach'
        result=dict(label=RUN_ID,run=dict(graph={'/t1/eval/obstacle':{'subscribers':['t1_evaluator']}},manifest=manifest),outcome='SUCCESS' if settle['settled'] else 'FAIL')
        # Record graph before shutdown, since live graph no longer exists after cleanup.
        result['run']['graph']['/t1/eval/obstacle']=self.runtime_graph['/t1/eval/obstacle']
        result['run']['graph']['/t1/cmd_teleop']=self.runtime_graph['/t1/cmd_teleop']
        ep.dynamic_evidence(type('RecordedRun',(),{'node':n})(),result,trace)
        self.summary['dynamic']=result['dynamic']
        contacts=self.summary['contacts']
        self.summary['arrived']=bool(success_sim and settle['settled'] and nav.get('verified') and
            settle['settle_start_sim']+2-nav['accepted_sim']<=120)
        graph=self.runtime_graph
        self.summary['graph_contract']=dict(final_only_collision_monitor=graph.get('/cmd_vel',{}).get('publishers')==['collision_monitor'],
            obstacle_truth_isolated=set(graph.get('/t1/eval/obstacle',{}).get('subscribers',[]))=={'t1_evaluator'},
            robot_truth_isolated=set(graph.get('/t1/eval/ground_truth',{}).get('subscribers',[]))=={'t1_evaluator'})
        exit_ok=all(v==0 for v in self.summary['parent_exit_codes'].values()) and bool(self.summary['launch_process_exit_codes']) and all(r['exit_code']==0 for r in self.summary['launch_process_exit_codes'].values()) and manifest.get('exit_code')==0
        self.summary['validation_before_port_check']=bool(self.summary['arrived'] and not self.summary['fell'] and
            contacts==dict(static_env=0,obstacle=0,ground_nonfoot=0) and result['dynamic']['verdict']=='PASS' and
            exit_ok and all(self.summary['graph_contract'].values()) and self.summary.get('safe_stop',{}).get('physical_stationary') and
            not self.summary['remaining_owned_pids'] and not self.summary['remaining_project_processes'] and
            self.summary.get('clock_regressions')==0 and
            len(self.summary['initializations'])==1 and not self.summary.get('error') and not self.summary.get('gpu_monitor_error'))
        self.summary['status']='PENDING_PORT_CHECK' if self.summary['validation_before_port_check'] else 'FAILED'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--startup-check',action='store_true',help='pre-registered 30s standing check; no navigation goal')
    parser.add_argument('--preflight',action='store_true')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--hold',action='store_true',help='hold the final view until Ctrl+C')
    parser.add_argument('--scenario',choices=('U01','sudden-approach'),default='U01')
    args=parser.parse_args()
    args.gpu_watch=None
    args.cpu_guard=None
    if args.headless or args.preflight:
        try:
            from cpu_guard import enter
            args.cpu_guard=enter()
            save('cpu-guard.json',args.cpu_guard)
        except Exception as exc:
            say(f'CPU isolation FAILED: {exc}; refusing all runtime imports')
            return 3
    else:
        try:
            profile=json.loads(GPU_PROFILE.read_text())
            for path,expected in profile['stages'].items():
                if sha(WS/path)!=expected:raise RuntimeError(f'GPU validation evidence changed: {path}')
            minimal = args.startup_check and args.no_rviz and profile['status']=='RECOVERY_MINIMAL_ONLY'
            if profile['status']!='GUI_VALIDATED' and not minimal:
                raise RuntimeError(f'{SAFETY_BLOCK_REASON} {profile.get("halt_reason", "")} Read docs/demo/release-manifest.md')
            from gpu_watch import Watch,GPU_ENV
            os.environ.update(GPU_ENV)
            args.gpu_watch=Watch(OUT/'gpu',idle_wait_s=60)
        except Exception as exc:
            say(f'GPU preflight refused: {exc}')
            return 3
    def stop(signum,frame):
        global STOP
        STOP=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    lock=(WS/'run_data/demo/active.lock').open('w')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        provenance=preflight(args)
    except Exception as exc:
        say(f'Preflight FAILED: {exc}'); save('preflight-error.json',dict(error=repr(exc),traceback=traceback.format_exc()))
        if args.gpu_watch:
            args.gpu_watch.halt(exc);args.gpu_watch.close()
        return 2
    if args.preflight:
        return 0
    demo=Demo(args,provenance)
    say(f'Run {RUN_ID} | detailed logs: {OUT}')
    try:
        demo.execute()
    except InterruptedError:
        demo.summary['human_intervention']=True
        demo.summary['error']='Interrupted before final view'
    except Exception as exc:
        demo.summary['error']=repr(exc)
        (OUT/'failure.txt').write_text(traceback.format_exc())
        say(f'FAILED: {exc}; no automatic retry')
    finally:
        demo.runtime_graph={}
        if demo.node:
            try:
                demo.runtime_graph=demo.node.graph(('/cmd_vel','/t1/eval/obstacle','/t1/eval/ground_truth','/t1/cmd_teleop'))
            except Exception:
                demo.summary['graph_error']=traceback.format_exc()
        try:
            demo.cleanup()
        except Exception:
            demo.summary['cleanup_error']=traceback.format_exc()
        demo.stop_gpu_watch()
        try:
            if args.startup_check:
                manifest=json.loads((WS/'run_data'/RUN_ID/'manifest.json').read_text())
                import csv
                rows=list(csv.DictReader((WS/'run_data'/RUN_ID/'sim_trace.csv').open()))
                demo.summary.update(contacts=manifest.get('contact_counts'),simulator_exit_code=manifest.get('exit_code'),
                    zero_policy=all(float(row['u_pol_x'])==0 and float(row['u_pol_wz'])==0 for row in rows),
                    fell=any(row['state'] in ('FALLEN','FAULT') for row in rows))
                ok=demo.summary.get('startup_check_completed') and demo.summary.get('safe_stop',{}).get('physical_stationary') and all(v==0 for v in demo.summary.get('parent_exit_codes',{}).values()) and all(r['exit_code']==0 for r in demo.summary.get('launch_process_exit_codes',{}).values()) and not demo.summary.get('remaining_owned_pids') and not demo.summary.get('error') and not demo.summary.get('gpu_monitor_error') and manifest.get('exit_code')==0 and demo.summary['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0) and demo.summary['zero_policy'] and not demo.summary['fell']
                demo.summary['validation_before_port_check']=bool(ok)
                demo.summary['status']='PENDING_PORT_CHECK' if ok else 'FAILED'
            else:
                demo.metrics()
        except Exception:
            demo.summary['metric_error']=traceback.format_exc()
            demo.summary['status']='FAILED'
        save('result.json',demo.summary)
        s=demo.summary
        say(f'{s["status"]} | arrived={s["arrived"]} | error={s.get("settle")} | contacts={s.get("contacts")} | fall={s.get("fell")} | sampled body gap={s.get("body_clearance",{}).get("min_m")}m')
        say(f'Exit codes {s.get("parent_exit_codes")} | remaining={s.get("remaining_owned_pids")} | port released={s.get("dds_port_released")} | result: {OUT}/result.json')
    return 4 if demo.summary['status']=='PENDING_PORT_CHECK' else 1


if __name__=='__main__':
    raise SystemExit(main())

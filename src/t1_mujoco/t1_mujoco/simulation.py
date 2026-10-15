"""ROS-facing entry point; physics lives only on this process's main thread."""
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

from .runtime import inspect_runtime, load_task_config
from .recording import RunLog


def parameter_defaults():
    return dict(upstream_path=os.getenv('T1_UPSTREAM', ''), assets_path=os.getenv('T1_ASSETS', ''),
                task='t1_walk', mode='sim_only', scene='', headless=True, viewer_rate_hz=30.0,
                target_rtf=1.0, enable_policy_cmd_input=False, cmd_timeout_sim=0.5,
                cmd_timeout_wall=0.5, warmup_sim_s=2.0, run_id='',
                run_dir=str(Path(os.getenv('T1_WS', Path.cwd()))/'run_data'), seed=0,
                max_sim_time=0.0, command_map_file='', joint_state_divisor=1,
                lidar_noise_std=0.0, lidar_dropout=0.0, lidar_publish_points=False,
                map_file='', slam_params_file='', localization_params_file='',
                safety_required=True, safety_params_file='', start_pose='',
                navigation_params_file='', events_file='', scenario='')


def validate_parameters(p):
    from .lidar_config import LidarConfig
    LidarConfig(noise_std=p['lidar_noise_std'], dropout=p['lidar_dropout'], seed=p['seed'])
    if p['mode'] not in ('sim_only', 'mapping', 'localization', 'navigation'):
        raise ValueError('evaluation requires W13')
    if p['mode'] != 'sim_only' and p['enable_policy_cmd_input']:
        raise ValueError('mapping/localization forbid policy debug commands')
    if p['mode'] in ('localization', 'navigation') and not p['map_file']:
        raise ValueError('localization/navigation require a saved map_file and matching metadata')
    if p['mode'] == 'navigation' and not (p['safety_required'] and p['navigation_params_file']):
        raise ValueError('navigation requires the W10 safety chain and navigation_params_file')
    from .scenes import parse_start_pose
    parse_start_pose(p['start_pose'])
    if bool(p['events_file']) != bool(p['scenario']):
        raise ValueError('events_file and scenario must be supplied together')
    if p['scenario'] and (p['mode'] not in ('sim_only', 'navigation')
                          or p['scene'] != 'dynamic_crossing_world'):
        raise ValueError('events only supported in dynamic_crossing_world sim_only/navigation; mapping excludes bodies')
    for key in ('map_file', 'slam_params_file', 'localization_params_file', 'navigation_params_file', 'events_file'):
        if p[key] and not Path(p[key]).is_absolute():
            raise ValueError(f'{key} must be absolute')
    if p['command_map_file'] and not Path(p['command_map_file']).is_absolute():
        raise ValueError('command_map_file must be absolute (empty = uncalibrated, /cmd_vel rejected)')
    for name in ('target_rtf','max_sim_time','warmup_sim_s','cmd_timeout_sim','cmd_timeout_wall','viewer_rate_hz'):
        if not math.isfinite(p[name]) or p[name] < 0:
            raise ValueError(f'{name} must be finite and nonnegative')
    for name in ('cmd_timeout_sim','cmd_timeout_wall','viewer_rate_hz'):
        if p[name] <= 0:
            raise ValueError(f'{name} must be positive')
    if p['viewer_rate_hz'] > 50:
        raise ValueError('viewer_rate_hz must be <=50')
    if p['run_id'] and (Path(p['run_id']).name != p['run_id'] or p['run_id'] in ('.','..')):
        raise ValueError('run_id must be a single directory name')
    if not Path(p['run_dir']).is_absolute():
        raise ValueError('run_dir must be absolute')
    if type(p['joint_state_divisor']) is not int or p['joint_state_divisor'] < 1:
        raise ValueError('joint_state_divisor must be a positive integer')


def main(args=None):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String, Bool
    from std_srvs.srv import SetBool, Trigger
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    context, node, executor, worker, runner, log, viewer = Context(), None, None, None, None, None, None
    contacts = None
    viewer_threads = []
    stop = threading.Event()
    old_signals = {}
    code, terminal = 0, 'SHUTDOWN'
    try:
        rclpy.init(args=args, context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = Node('t1_sim', context=context)
        if node.get_parameter('use_sim_time').value:
            raise ValueError('t1_sim must use_sim_time=false; stamps come explicitly from data.time')
        node.set_parameters([Parameter('use_sim_time', value=False)])
        # Runtime mutation could break timing/ownership. Relaunch for parameter changes.
        p = {}
        for name, default in parameter_defaults().items():
            p[name] = node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
        node.add_on_set_parameters_callback(lambda _: SetParametersResult(successful=False, reason='restart t1_sim to change parameters'))
        validate_parameters(p)
        runtime = inspect_runtime(p['upstream_path'], p['assets_path'], p['mode'], p['task'])
        cfg = load_task_config(p['upstream_path'], p['task'])
        from ament_index_python.packages import get_package_share_directory
        from .scenes import configure_scene, parse_start_pose
        scenario, event_provenance = None, None
        if p['scenario']:
            from .events import load_scenario
            scenario, event_provenance = load_scenario(p['events_file'], p['scenario'])
        scene_provenance = configure_scene(
            cfg, p['scene'], runtime['assets_path'], Path(p['run_dir'])/'cache/scenes',
            Path(get_package_share_directory('t1_mujoco'))/'worlds', parse_start_pose(p['start_pose']),
            active_slot=scenario['slot'] if scenario else None)
        if scene_provenance:
            runtime['mjcf_path'] = scene_provenance['expanded_xml']['path']
        import torch
        import numpy as np
        torch.set_num_threads(1)
        torch.manual_seed(p['seed'])
        np.random.seed(p['seed'])
        command_map = None
        if p['command_map_file']:
            from .command_map import CommandMap
            command_map = CommandMap.load(p['command_map_file'])  # invalid map: refuse to start
        elif p['mode'] != 'sim_only':
            raise ValueError('navigation/evaluation modes require a calibrated command_map_file')
        log = RunLog(p, runtime)
        from .recording import w09_provenance
        provenance = w09_provenance(p, scene_provenance)
        log.manifest['parameter_files'].update(provenance.pop('parameter_files'))
        log.manifest.update(provenance)
        if scene_provenance:
            log.manifest['scene_provenance'] = scene_provenance
            metadata = scene_provenance['scene_metadata']
            log.manifest['goal'] = metadata['goal'] if metadata else None
            log.manifest['case'] = metadata['name'] if metadata else 'custom_mjcf'
            log.manifest['start_pose_override'] = scene_provenance.get('start_pose_override')
            log.save()
        if command_map is not None:
            from .recording import sha256
            log.manifest['parameter_files']['command_map'] = dict(
                path=command_map.source, sha256=sha256(command_map.source),
                evidence=command_map.data['evidence'])
        clock_pub = node.create_publisher(Clock, '/clock', QoSProfile(depth=1))
        state_pub = node.create_publisher(String, '/t1/sim_state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        diag_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        from .state_bridge import StatePublisher, capture, stamp
        state_bridge = StatePublisher(node, p['joint_state_divisor'])
        from .runner import T1SimRunner
        from .model_contract import ModelIncompatible
        try:
            runner = T1SimRunner(cfg, debug=p['enable_policy_cmd_input'], warmup=p['warmup_sim_s'],
                                 timeout_sim=p['cmd_timeout_sim'], timeout_wall=p['cmd_timeout_wall'],
                                 command_map=command_map)
        except (ModelIncompatible, ValueError) as exc:
            status = dict(schema=1, state='FAULT', fault='model_incompatible', detail=str(exc),
                          sim_time=0.0, run_id=log.run_id, wall_unix=time.time())
            state_pub.publish(String(data=json.dumps(status)))
            log.event(status)
            raise
        from dataclasses import asdict
        from .lidar_config import LidarConfig
        from .lidar import RayLidar, LidarPublisher
        lidar_cfg = LidarConfig(noise_std=p['lidar_noise_std'], dropout=p['lidar_dropout'],
                                seed=p['seed'], publish_points=p['lidar_publish_points'])
        lidar = RayLidar(runner, lidar_cfg)
        runner.sensor_required = True
        runner.sensor_healthy = False  # no motion until first valid scan
        lidar_pub = LidarPublisher(node, lidar_cfg)
        log.manifest.update(sensor_mode='compensated', lidar_config=asdict(lidar_cfg))
        log.manifest['initial_pose'] = runner.mj_data.qpos[:7].tolist()
        from .contacts import ContactMonitor
        contacts = ContactMonitor(runner.mj_model, log.path/'contact_events.csv', cfg.policy_dt)
        runner.substep_hook = contacts.sample
        event_pub = None
        if scenario:
            from .events import SceneEvents
            runner.scene_events = SceneEvents(runner, scenario, log)
            event_pub = node.create_publisher(String, '/t1/eval/obstacle', 10)
            log.manifest['parameter_files']['events'] = event_provenance
        log.manifest['model_dimensions'] = dict(nq=runner.mj_model.nq, nv=runner.mj_model.nv,
                                                 nu=runner.mj_model.nu, nmocap=runner.mj_model.nmocap)
        log.save()

        runner.inputs.safety_required = p['safety_required']
        if p['safety_required']:
            node.create_subscription(Bool, '/t1/safety/permit',
                                     lambda msg: runner.inputs.safety(msg.data), 1)
        if p['safety_params_file']:
            from .recording import sha256
            for key in ('velocity_smoother', 'collision_monitor', 'command_gate'):
                log.manifest['parameter_files'][key] = dict(path=p['safety_params_file'], sha256=sha256(p['safety_params_file']))
            log.save()

        def command(msg):
            runner.inputs.receive((msg.linear.x, msg.linear.y, msg.linear.z,
                                   msg.angular.x, msg.angular.y, msg.angular.z))
        node.create_subscription(Twist, '/t1/debug/policy_cmd' if runner.debug else '/cmd_vel', command, QoSProfile(depth=1))

        def pause(request, response):
            response.success, response.message = runner.requests.submit('pause', request.data)
            return response

        def reset(_request, response):
            response.success, response.message = runner.requests.submit('reset')
            return response
        node.create_service(SetBool, '/t1/sim/pause', pause)
        node.create_service(Trigger, '/t1/sim/reset', reset)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, name='t1_ros_callbacks')
        worker.start()
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_signals[sig] = signal.signal(sig, lambda *_: stop.set())

        if not p['headless']:
            import mujoco.viewer
            # Render a COPY: viewer UI never owns or modifies the physical state.
            view_model = mujoco.MjModel.from_xml_path(runtime['mjcf_path'])
            view_model.opt.timestep = cfg.mujoco.physics_dt
            view_data = mujoco.MjData(view_model)
            demo_presentation = os.environ.get('T1_DEMO_PRESENTATION') == '1'
            if demo_presentation:
                from .presentation import style_model, decorate
                style_model(view_model)
            def key(code):
                if code == 32:
                    from .control import Request
                    with runner.inputs.lock:
                        paused = runner.inputs.state == 'PAUSED'
                    runner.requests.queue.put(Request('pause', not paused))
            before_viewer_threads = set(threading.enumerate())
            viewer = mujoco.viewer.launch_passive(view_model, view_data, key_callback=key,
                                                  show_left_ui=False, show_right_ui=False)
            viewer_threads = [t for t in threading.enumerate() if t not in before_viewer_threads]
            viewer.cam.elevation = -20
            if demo_presentation:
                with viewer.lock():
                    decorate(viewer)

        samples = deque()
        rtf = 0.0
        last_heartbeat = last_view = 0.0
        deadline = time.monotonic()
        overruns = 0
        status_signature = None

        def publish_clock(ns):
            msg = Clock()
            stamp(msg.clock, ns)
            clock_pub.publish(msg)

        def status(force=False):
            nonlocal last_heartbeat, status_signature
            now = time.monotonic()
            signature = (runner.state, runner.fault, runner.cmd_status, runner.episode, runner.sensor_healthy)
            if not force and signature == status_signature and now-last_heartbeat < 1:
                return
            last_heartbeat, status_signature = now, signature
            value = dict(schema=1, run_id=log.run_id, episode=runner.episode, state=runner.state,
                         fault=runner.fault, mode=p['mode'], sim_time=float(runner.mj_data.time),
                         wall_unix=time.time(), wall_mono=now, rtf_10s=rtf, cmd_status=runner.cmd_status,
                         cmd_source='policy_cmd' if runner.debug else 'cmd_vel',
                         v_des=[0.,0.,0.] if runner.debug else list(runner.incoming), u_pol=list(runner.u_pol),
                         v_guard=[0.,0.,0.] if runner.debug else list(runner.v_guard), guard=list(runner.guard_flags),
                         command_map_calibrated=runner.command_map is not None,
                         scenario=runtime['mjcf_path'], seed=p['seed'], code_sha=log.manifest['code_sha'],
                         deadline_overruns=overruns, ignored_components=runner.inputs.ignored_components,
                         odom_mode='ideal_truth', covariance_mode='nominal_not_estimated',
                         sensor_mode='compensated', sensor_healthy=runner.sensor_healthy,
                         scan_compute_ms=runner.scan_compute_ms, scan_known_fraction=runner.scan_known_fraction,
                         scan_blind_bins=runner.scan_blind_bins,
                         env_contacts=dict(contacts.counts))
            state_pub.publish(String(data=json.dumps(value, allow_nan=False)))
            log.event(value)
            diag = DiagnosticArray()
            ns = round(float(runner.mj_data.time)*1_000_000_000)
            diag.header.stamp.sec, diag.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            item = DiagnosticStatus(name='t1_sim', hardware_id='mujoco_t1', message=runner.fault or ('sensor_coverage_blocked' if not runner.sensor_healthy else runner.cmd_status),
                                    level=DiagnosticStatus.ERROR if runner.state in ('FAULT','FALLEN') or not runner.sensor_healthy else (DiagnosticStatus.WARN if runner.cmd_status in ('UNCALIBRATED','INVALID','TIMEOUT') else DiagnosticStatus.OK))
            item.values = [KeyValue(key='state', value=runner.state), KeyValue(key='policy_dt', value='0.02'),
                           KeyValue(key='odom_mode', value='ideal_truth'),
                           KeyValue(key='covariance_mode', value='nominal_not_estimated'),
                           KeyValue(key='sensor_mode', value='compensated'),
                           KeyValue(key='sensor_healthy', value=str(runner.sensor_healthy)),
                           KeyValue(key='scan_known_fraction', value=str(runner.scan_known_fraction)),
                           KeyValue(key='scan_blind_bins', value=str(runner.scan_blind_bins))]
            diag.status = [item]
            diag_pub.publish(diag)
            for event in runner.state_events:
                log.event(dict(transition=event[0], fault=event[1], sim_time=event[2], episode=event[3], wall_mono=now))
            runner.state_events.clear()

        publish_clock(capture(runner).stamp_ns)
        status(True)
        print('T1_RUN '+str(log.path), flush=True)
        while not stop.is_set() and context.ok():
            if viewer is not None and not viewer.is_running():
                break
            runner.handle_requests(p['mode'])
            before = time.monotonic()
            if runner.step():
                samples.append((before, float(runner.mj_data.time)))
                while len(samples)>2 and before-samples[0][0]>10:
                    samples.popleft()
                if len(samples)>1:
                    rtf = (samples[-1][1]-samples[0][1])/(samples[-1][0]-samples[0][0])
                snapshot = capture(runner)
                publish_clock(snapshot.stamp_ns)
                state_bridge.publish(snapshot, runner.episode)
                runner.scan_due = lidar.due(runner.episode)
                runner.scan_compute_ms = 0.0
                if runner.scan_due:
                    try:
                        frame = lidar.sample(snapshot)
                        lidar_pub.publish(frame)
                        runner.sensor_healthy = frame.healthy
                        runner.scan_compute_ms = frame.compute_ms
                        runner.scan_known_fraction = frame.known_fraction
                        runner.scan_blind_bins = frame.blind_bins
                    except Exception as exc:
                        runner.sensor_healthy = False
                        runner.failure_detail = repr(exc)
                        runner.transition('FAULT', 'sensor_exception')
                if event_pub is not None:
                    event_pub.publish(String(data=json.dumps(runner.scene_events.snapshot())))
                log.row(runner, rtf)
                runner.finish_boundary()
            status()
            if viewer is not None and before-last_view >= 1/p['viewer_rate_hz']:
                import mujoco
                with viewer.lock():
                    mujoco.mj_copyData(view_data, view_model, runner.mj_data)
                    if not demo_presentation:
                        viewer.cam.lookat[:] = runner.mj_data.qpos[:3]
                viewer.sync(state_only=True)
                last_view = before
            if runner.state in ('FAULT', 'FALLEN'):
                code, terminal = 2, runner.state
                break
            if p['max_sim_time'] and runner.mj_data.time + 1e-9 >= p['max_sim_time']:
                break
            if runner.state == 'PAUSED':
                stop.wait(0.005)
                deadline = time.monotonic()
            elif p['target_rtf']:
                deadline += cfg.policy_dt/p['target_rtf']
                remaining = deadline-time.monotonic()
                if remaining > 0:
                    stop.wait(remaining)
                else:
                    overruns += 1
                    deadline = time.monotonic()  # no catch-up burst or skipped physics
        if runner.had_failure:
            code = 2
        if code == 0:
            runner.transition('SHUTDOWN')
            status(True)
        log.manifest.update(deadline_overruns=overruns, physics_steps=runner._step_count*runner.decimation,
                            failure_detail=getattr(runner,'failure_detail',None))
    except Exception as exc:
        code, terminal = 2, 'FAULT'
        print(f'T1 simulation failed: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        if log is not None:
            log.manifest['failure_detail'] = repr(exc)
        if runner is not None:
            runner.transition('FAULT', 'internal_error')
    finally:
        stop.set()
        if runner is not None:
            runner.write_command((0.,0.,0.))
            runner.stop()
        if viewer is not None:
            viewer.close()
            # MuJoCo 3.15 close() only requests exit; its render thread is daemon.
            # Wait for the threads created by this launch before Python teardown.
            for thread in viewer_threads:
                thread.join(timeout=5)
                if thread.is_alive():
                    code = 2
                    print('viewer thread failed to exit', file=sys.stderr)
        if executor is not None:
            executor.shutdown(timeout_sec=3)
        if worker is not None:
            worker.join(timeout=3)
            if worker.is_alive():
                code = 2
                print('ROS callback thread did not terminate', file=sys.stderr)
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
        if contacts is not None:
            contacts.close()
            if log is not None:
                log.manifest['contact_counts'] = dict(contacts.counts)
        if log is not None:
            log.close(code, float(runner.mj_data.time) if runner else 0.0, terminal)
    return code


if __name__ == '__main__':
    sys.exit(main())

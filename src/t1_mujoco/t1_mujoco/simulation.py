"""ROS-facing entry point; physics lives only on this process's main thread."""
from collections import deque
from contextlib import nullcontext
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
                max_sim_time=0.0, command_map_file='')


def validate_parameters(p):
    if p['mode'] != 'sim_only':
        raise ValueError('W04 implements sim_only only; mapping/navigation/evaluation are not available')
    if p['command_map_file']:
        raise ValueError('command_map_file is implemented by W05, not W04')
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
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    context, node, executor, worker, runner, log, viewer = Context(), None, None, None, None, None, None
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
        if p['scene']:
            scene = Path(p['scene'])
            if not scene.is_absolute() or not scene.is_file():
                raise ValueError('scene must be an existing absolute MJCF path')
            cfg.robot = cfg.robot.replace(mjcf_path=str(scene))
            runtime['mjcf_path'] = str(scene)
        import torch
        import numpy as np
        torch.set_num_threads(1)
        torch.manual_seed(p['seed'])
        np.random.seed(p['seed'])
        log = RunLog(p, runtime)
        clock_pub = node.create_publisher(Clock, '/clock', QoSProfile(depth=1))
        state_pub = node.create_publisher(String, '/t1/sim_state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        diag_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        from .runner import T1SimRunner
        from .model_contract import ModelIncompatible
        try:
            runner = T1SimRunner(cfg, debug=p['enable_policy_cmd_input'], warmup=p['warmup_sim_s'],
                                 timeout_sim=p['cmd_timeout_sim'], timeout_wall=p['cmd_timeout_wall'])
        except (ModelIncompatible, ValueError) as exc:
            status = dict(schema=1, state='FAULT', fault='model_incompatible', detail=str(exc),
                          sim_time=0.0, run_id=log.run_id, wall_unix=time.time())
            state_pub.publish(String(data=json.dumps(status)))
            log.event(status)
            raise
        log.manifest['initial_pose'] = runner.mj_data.qpos[:7].tolist()
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
            def key(code):
                if code == 32:
                    from .control import Request
                    with runner.inputs.lock:
                        paused = runner.inputs.state == 'PAUSED'
                    runner.requests.queue.put(Request('pause', not paused))
            viewer = mujoco.viewer.launch_passive(view_model, view_data, key_callback=key,
                                                  show_left_ui=False, show_right_ui=False)
            viewer.cam.elevation = -20

        samples = deque()
        rtf = 0.0
        last_heartbeat = last_view = 0.0
        deadline = time.monotonic()
        overruns = 0
        status_signature = None

        def publish_clock():
            ns = round(float(runner.mj_data.time)*1_000_000_000)
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(ns, 1_000_000_000)
            clock_pub.publish(msg)

        def status(force=False):
            nonlocal last_heartbeat, status_signature
            now = time.monotonic()
            signature = (runner.state, runner.fault, runner.cmd_status, runner.episode)
            if not force and signature == status_signature and now-last_heartbeat < 1:
                return
            last_heartbeat, status_signature = now, signature
            value = dict(schema=1, run_id=log.run_id, episode=runner.episode, state=runner.state,
                         fault=runner.fault, mode=p['mode'], sim_time=float(runner.mj_data.time),
                         wall_unix=time.time(), wall_mono=now, rtf_10s=rtf, cmd_status=runner.cmd_status,
                         cmd_source='policy_cmd' if runner.debug else 'cmd_vel',
                         v_des=[0.,0.,0.] if runner.debug else list(runner.incoming), u_pol=list(runner.u_pol),
                         scenario=runtime['mjcf_path'], seed=p['seed'], code_sha=log.manifest['code_sha'],
                         deadline_overruns=overruns, ignored_components=runner.inputs.ignored_components)
            state_pub.publish(String(data=json.dumps(value, allow_nan=False)))
            log.event(value)
            diag = DiagnosticArray()
            ns = round(float(runner.mj_data.time)*1_000_000_000)
            diag.header.stamp.sec, diag.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            item = DiagnosticStatus(name='t1_sim', hardware_id='mujoco_t1', message=runner.fault or runner.cmd_status,
                                    level=DiagnosticStatus.ERROR if runner.state in ('FAULT','FALLEN') else (DiagnosticStatus.WARN if runner.cmd_status in ('UNCALIBRATED','INVALID','TIMEOUT') else DiagnosticStatus.OK))
            item.values = [KeyValue(key='state', value=runner.state), KeyValue(key='policy_dt', value='0.02')]
            diag.status = [item]
            diag_pub.publish(diag)
            for event in runner.state_events:
                log.event(dict(transition=event[0], fault=event[1], sim_time=event[2], episode=event[3], wall_mono=now))
            runner.state_events.clear()

        publish_clock()
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
                publish_clock()
                log.row(runner, rtf)
                runner.finish_boundary()
            status()
            if viewer is not None and before-last_view >= 1/p['viewer_rate_hz']:
                import mujoco
                with viewer.lock():
                    mujoco.mj_copyData(view_data, view_model, runner.mj_data)
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
        if log is not None:
            log.close(code, float(runner.mj_data.time) if runner else 0.0, terminal)
    return code


if __name__ == '__main__':
    sys.exit(main())

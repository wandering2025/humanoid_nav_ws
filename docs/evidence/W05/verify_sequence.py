"""W05 C4: fixed 120 s physical /cmd_vel sequences through the real launch entry.

Starts its own loopback discovery server (ADR-009), then for each run launches
`ros2 launch t1_bringup simulation.launch.py` with the installed calibrated
command_map.yaml, streams geometry_msgs/Twist on /cmd_vel keyed on /clock
(sim time since READY), and analyses the run's sim_trace.csv.
Writes docs/evidence/W05/c4-sequence.json. Never touches physical state.

usage: python verify_sequence.py [--runs 3]
"""
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

WS = Path(os.environ['T1_WS'])
OUT = WS / 'docs/evidence/W05'
DURATION = 120.0
# (start s after READY, v_des = (vx m/s, vy, wz rad/s), label). Physical units.
SEQUENCE = [
    (0.0, (0.0, 0.0, 0.0), 'stand'),
    (5.0, (0.40, 0.0, 0.0), 'forward_0.40'),
    (20.0, (0.40, 0.0, 0.30), 'forward_left'),
    (28.0, (0.50, 0.0, -0.30), 'forward_right'),
    (36.0, (0.0, 0.0, 0.0), 'stop_1'),
    (41.0, (0.0, 0.0, 0.60), 'turn_in_place_left'),
    (49.0, (0.0, 0.0, 0.0), 'stop_2'),
    (54.0, (0.0, 0.0, -0.60), 'turn_in_place_right'),
    (62.0, (0.0, 0.0, 0.0), 'stop_3'),
    (67.0, (0.60, 0.0, 0.0), 'forward_0.60'),
    (82.0, (0.32, 0.0, 0.15), 'slow_forward_left'),
    (88.0, (0.0, 0.0, 0.0), 'stop_4'),
    (93.0, (0.15, 0.0, 0.0), 'deadband_forward_0.15'),
    (100.0, (0.45, 0.0, -0.40), 'forward_right_0.40'),
    (110.0, (0.0, 0.0, 0.0), 'stop_final'),
]
STILL_SPEED, STILL_RATE, WINDOW, HOLD = 0.05, 0.05, 0.5, 2.0


def segment_at(t):
    current = SEQUENCE[0]
    for item in SEQUENCE:
        if t + 1e-9 >= item[0]:
            current = item
    return current


class Driver(Node):
    def __init__(self, run_id):
        super().__init__('w05_sequence')
        self.run_id, self.state, self.sim, self.ready_time = run_id, None, 0.0, None
        self.create_subscription(String, '/t1/sim_state', self.on_state,
                                 QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Clock, '/clock', self.on_clock, 100)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sent = 0

    def on_state(self, msg):
        value = json.loads(msg.data)
        if value.get('run_id') == self.run_id:
            self.state = value
            if value.get('state') == 'READY' and self.ready_time is None:
                self.ready_time = value['sim_time']

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def publish(self):
        if self.ready_time is None:
            return
        _, (vx, vy, wz), _ = segment_at(self.sim - self.ready_time)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(msg)
        self.sent += 1


def analyse(path):
    rows = list(csv.DictReader((path / 'sim_trace.csv').open()))
    f = lambda r, k: float(r[k])
    ready = [r for r in rows if r['state'] == 'READY']
    t_ready = f(ready[0], 'sim_time') - 0.02
    times = [f(r, 'sim_time') for r in rows]
    assert all(b > a for a, b in zip(times, times[1:])), 'sim time not monotonic'
    finite = all(math.isfinite(f(r, k)) for r in rows for k in ('x_w', 'y_w', 'z_w', 'yaw_w', 'u_pol_x', 'u_pol_wz'))
    n = int(round(WINDOW / 0.02))
    segments = []
    for i, (start, v, label) in enumerate(SEQUENCE):
        end = SEQUENCE[i + 1][0] if i + 1 < len(SEQUENCE) else DURATION
        sub = [r for r in ready if start <= f(r, 'sim_time') - t_ready - 0.02 + 1e-9 < end]
        if not sub:
            segments.append(dict(label=label, rows=0))
            continue
        k0 = rows.index(sub[0])
        x0, y0 = f(rows[k0 - 1], 'x_w'), f(rows[k0 - 1], 'y_w')
        yaw = [f(r, 'yaw_w') for r in rows[k0 - 1:k0 + len(sub)]]
        yaw = [yaw[0]] + [yaw[0] + sum(math.remainder(b - a, math.tau) for a, b in zip(yaw[:j], yaw[1:j + 1]))
                          for j in range(1, len(yaw))]
        half = sub[len(sub) // 2:]
        status = {}
        for r in sub:
            status[r['cmd_status']] = status.get(r['cmd_status'], 0) + 1
        item = dict(label=label, v_des=v, t_start=start, t_end=end, rows=len(sub), cmd_status=status,
                    u_pol_last=[f(sub[-1], 'u_pol_' + a) for a in ('x', 'y', 'wz')],
                    v_guard_last=[f(sub[-1], 'v_guard_' + a) for a in ('x', 'y', 'wz')],
                    mean_vx_2nd_half=sum(f(r, 'v_meas_x') for r in half) / len(half),
                    mean_wz_2nd_half=(yaw[-1] - yaw[len(yaw) - len(half) - 1]) / (len(half) * 0.02),
                    displacement_m=math.hypot(f(sub[-1], 'x_w') - x0, f(sub[-1], 'y_w') - y0),
                    yaw_change_deg=math.degrees(yaw[-1] - yaw[0]),
                    min_z=min(f(r, 'z_w') for r in sub), max_proj_grav_z=max(f(r, 'proj_grav_z') for r in sub))
        if v == (0.0, 0.0, 0.0) and i > 0:
            seg_rows = rows[k0 - 1:k0 + len(sub)]
            still = []
            for j in range(1, len(seg_rows)):
                a = seg_rows[max(0, j - n)]
                b = seg_rows[j]
                dt = f(b, 'sim_time') - f(a, 'sim_time')
                sp = math.hypot(f(b, 'x_w') - f(a, 'x_w'), f(b, 'y_w') - f(a, 'y_w')) / dt
                rt = abs(yaw[j] - yaw[max(0, j - n)]) / dt
                still.append(j >= n and sp < STILL_SPEED and rt < STILL_RATE)
            hold = int(round(HOLD / 0.02))
            settle = next((j for j in range(len(still) - hold + 1) if all(still[j:j + hold])), None)
            item.update(settle_s=(settle + 1) * 0.02 if settle is not None else None,
                        stop_dist_max_m=max(math.hypot(f(r, 'x_w') - x0, f(r, 'y_w') - y0) for r in sub),
                        stop_yaw_deg=math.degrees(yaw[-1] - yaw[0]), settled_and_held_2s=settle is not None)
        segments.append(item)
    return dict(rows=len(rows), ready_rows=len(ready), t_ready=t_ready, sim_end=times[-1], finite=finite,
                states=sorted({r['state'] for r in rows}), min_z=min(f(r, 'z_w') for r in rows),
                max_proj_grav_z=max(f(r, 'proj_grav_z') for r in rows),
                control_rtf=(times[-1] - times[0]) / (f(rows[-1], 'wall_mono') - f(rows[0], 'wall_mono')),
                path_xy=[[round(f(r, 'x_w'), 3), round(f(r, 'y_w'), 3)] for r in rows[::50]],
                segments=segments)


def run_once(index, port):
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + f'_dev_w05_seq{index}_s0_' + uuid.uuid4().hex[:6]
    max_sim = 2.0 + DURATION + 0.5
    cmd = ['ros2', 'launch', 't1_bringup', 'simulation.launch.py', 'target_rtf:=1.0',
           f'max_sim_time:={max_sim}', f'run_id:={run_id}']
    raw = OUT / 'raw'
    raw.mkdir(exist_ok=True)
    log = (raw / f'{run_id}.txt').open('w')
    env = dict(os.environ)
    env.pop('DISPLAY', None)
    env.pop('WAYLAND_DISPLAY', None)
    began, wall0 = datetime.now(timezone.utc).isoformat(), time.monotonic()
    proc = subprocess.Popen(cmd, cwd=WS, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, env=env)
    node = Driver(run_id)
    try:
        deadline = time.monotonic() + max_sim / 0.8 + 30
        next_pub = time.monotonic()
        while proc.poll() is None:
            assert time.monotonic() < deadline, 'run exceeded wall deadline'
            rclpy.spin_once(node, timeout_sec=0.005)
            if time.monotonic() >= next_pub:
                node.publish()
                next_pub += 0.02  # 50 Hz wall at target_rtf 1.0
        code = proc.wait()
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
        log.close()
        sent = node.sent
        node.destroy_node()
    path = WS / 'run_data' / run_id
    manifest = json.loads((path / 'manifest.json').read_text())
    result = dict(run_id=run_id, command=cmd, cwd=str(WS), started_utc=began, ended_utc=datetime.now(timezone.utc).isoformat(),
                  wall_s=time.monotonic() - wall0, exit_code=code, messages_sent=sent, log=f'raw/{run_id}.txt',
                  manifest_exit_code=manifest['exit_code'], code_sha=manifest['code_sha'], dirty=manifest['dirty_files'],
                  executable=manifest['executable'], command_map=manifest['parameter_files']['command_map'])
    result.update(analyse(path))
    return result


def verdict(run):
    checks = {
        'launch_exit_0': run['exit_code'] == 0 and run['manifest_exit_code'] == 0,
        'venv_interpreter': run['executable'] == os.environ['T1_VENV'] + '/bin/python',
        'calibrated_map_loaded': bool(run['command_map']),
        'full_120s': run['sim_end'] - run['t_ready'] >= DURATION - 0.02,
        'no_fall_or_fault': set(run['states']) <= {'STARTING', 'READY'} and run['max_proj_grav_z'] <= -0.5,
        'finite': run['finite'],
    }
    seg = {s['label']: s for s in run['segments']}
    checks['every_stop_settled_and_held_2s'] = all(s.get('settled_and_held_2s') for s in run['segments'] if 'settle_s' in s)
    checks['forward_moves'] = seg['forward_0.40']['mean_vx_2nd_half'] > 0.2 and seg['forward_0.60']['mean_vx_2nd_half'] > 0.4
    checks['left_turn_while_walking'] = seg['forward_left']['mean_wz_2nd_half'] > 0.15 and seg['forward_left']['mean_vx_2nd_half'] > 0.2
    checks['right_turn_while_walking'] = seg['forward_right']['mean_wz_2nd_half'] < -0.15 and seg['forward_right']['mean_vx_2nd_half'] > 0.2
    checks['turn_in_place_both'] = seg['turn_in_place_left']['yaw_change_deg'] > 90 and seg['turn_in_place_right']['yaw_change_deg'] < -90
    checks['deadband_not_raised'] = seg['deadband_forward_0.15']['u_pol_last'] == [0.0, 0.0, 0.0] and \
        seg['deadband_forward_0.15']['displacement_m'] < 0.1
    active = sum(s['cmd_status'].get('ACTIVE', 0) for s in run['segments'])
    timeouts = sum(s['cmd_status'].get('TIMEOUT', 0) for s in run['segments'])
    checks['commands_fresh'] = timeouts <= 0.01 * active
    return checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=3)
    args = parser.parse_args()
    assert os.environ.get('T1_LOCAL_DDS') == '1'
    port = os.environ['T1_DDS_PORT']
    started = datetime.now(timezone.utc).isoformat()
    server_log = (OUT / 'raw/c4-dds.txt').open('w')
    server = subprocess.Popen([str(WS / 'scripts/start_local_dds.sh')], stdout=server_log, stderr=subprocess.STDOUT)
    time.sleep(0.5)
    assert server.poll() is None, 'discovery server failed to start'
    runs = []
    rclpy.init()
    try:
        for i in range(args.runs):
            run = run_once(i, port)
            run['checks'] = verdict(run)
            runs.append(run)
            print(run['run_id'], run['exit_code'], all(run['checks'].values()), run['checks'], flush=True)
    finally:
        rclpy.shutdown()
        server.send_signal(signal.SIGINT)
        server.wait(timeout=5)
        server_log.close()
    result = dict(started_utc=started, ended_utc=datetime.now(timezone.utc).isoformat(), sequence=SEQUENCE,
                  stillness=dict(window_s=WINDOW, speed=STILL_SPEED, yaw_rate=STILL_RATE, hold_s=HOLD),
                  environment={k: os.environ.get(k) for k in ('T1_WS', 'T1_VENV', 'T1_UPSTREAM', 'T1_LOCAL_DDS', 'T1_DDS_PORT',
                                                            'ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_DISCOVERY_SERVER',
                                                            'ROS_SUPER_CLIENT', 'ROS_LOG_DIR')},
                  server_exit=server.returncode, runs=runs,
                  verdict='PASS' if len(runs) == args.runs and all(all(r['checks'].values()) for r in runs) else 'FAIL')
    (OUT / 'c4-sequence.json').write_text(json.dumps(result, indent=2) + '\n')
    print('VERDICT', result['verdict'])
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

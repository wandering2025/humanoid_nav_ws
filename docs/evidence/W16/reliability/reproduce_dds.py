"""Isolated DDS shutdown diagnostic; source scripts/env.sh before invoking.

No robot or GUI. Uses distinct localhost ports and only cleans up its children.
Results are fail-fast; output must be a new directory under ignored raw data.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

CLIENT = '''import rclpy,time
from std_msgs.msg import String
rclpy.init()
nodes=[rclpy.create_node('w16_churn_'+str(i)) for i in range(16)]
pubs=[n.create_publisher(String,'/w16_churn_'+str(i),10) for i,n in enumerate(nodes)]
subs=[n.create_subscription(String,'/w16_churn_'+str(i),lambda m:None,10) for i,n in enumerate(nodes)]
for j in range(40):
 for p in pubs:p.publish(String(data='diagnostic'))
 for n in nodes:rclpy.spin_once(n,timeout_sec=0)
 time.sleep(.05)
for n in nodes:n.destroy_node()
rclpy.shutdown()
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port-base', type=int, default=42130)
    parser.add_argument('--count', type=int, default=15)
    parser.add_argument('--drain', action='store_true')
    parser.add_argument('--backtrace-library', type=Path)
    args = parser.parse_args()
    if not 1 <= args.count <= 30 or not 1024 <= args.port_base <= 65535-args.count:
        parser.error('Require count 1..30 and a valid nonprivileged port range')
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for index in range(1, args.count+1):
        port = args.port_base+index
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', port))
        env = dict(os.environ, ROS_DISCOVERY_SERVER=f'TCPv4:[127.0.0.1]:{port}',
                   ROS_SUPER_CLIENT='TRUE', ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT',
                   ROS_DOMAIN_ID='38')
        server_env = dict(env)
        if args.backtrace_library:
            server_env['LD_PRELOAD'] = str(args.backtrace_library.resolve(strict=True))
        server = None
        with (args.output/f'{index:02}.log').open('w') as log:
            try:
                server = subprocess.Popen(['fast-discovery-server', '-i', '0', '-t',
                    '127.0.0.1', '-q', str(port)], env=server_env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                time.sleep(1)
                client = subprocess.run([sys.executable, '-c', CLIENT], env=env,
                    stdout=log, stderr=subprocess.STDOUT, timeout=40)
                began, quiet_since, tcp_threads = time.monotonic(), None, []
                if args.drain:
                    while time.monotonic()-began < 15 and server.poll() is None:
                        names = []
                        for task in Path(f'/proc/{server.pid}/task').iterdir():
                            try:
                                names.append((task/'comm').read_text().strip())
                            except FileNotFoundError:
                                pass
                        tcp_threads = [n for n in names if n.startswith('dds.tcp.')]
                        if tcp_threads:
                            quiet_since = None
                        elif quiet_since is None:
                            quiet_since = time.monotonic()
                        elif time.monotonic()-quiet_since >= .5:
                            break
                        time.sleep(.05)
                else:
                    time.sleep(1)
                if server.poll() is None:
                    server.send_signal(signal.SIGINT)
                rc = server.wait(timeout=10)
                row = dict(index=index, port=port, client_exit=client.returncode,
                           server_exit=rc, drain_requested=args.drain,
                           tcp_threads=tcp_threads, wait_and_shutdown_s=time.monotonic()-began)
                results.append(row)
                (args.output/'results.json').write_text(json.dumps(results, indent=2)+'\n')
                print(json.dumps(row), flush=True)
            finally:
                if server and server.poll() is None:
                    server.send_signal(signal.SIGINT)
                    try:
                        server.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        server.kill()
                        server.wait()
        if rc or client.returncode or tcp_threads:
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

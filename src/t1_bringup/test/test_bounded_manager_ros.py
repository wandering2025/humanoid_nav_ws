"""Real service-boundary tests for the installed bounded manager (no simulation)."""
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from nav2_msgs.srv import ManageLifecycleNodes
from bond.msg import Status
from std_msgs.msg import Header


@pytest.fixture
def manager(tmp_path):
    # Own a separate discovery server; never reuse or stop a user's server.
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='134', ROS_DISCOVERY_SERVER='TCPv4:[127.0.0.1]:42126',
               ROS_SUPER_CLIENT='TRUE', ROS_AUTOMATIC_DISCOVERY_RANGE='SYSTEM_DEFAULT')
    previous = {k: os.environ.get(k) for k in env if k.startswith('ROS_')}
    os.environ.update({k: v for k, v in env.items() if k.startswith('ROS_')})
    log = (tmp_path/'dds.log').open('w')
    dds = subprocess.Popen(['fast-discovery-server', '-i', '0', '-t', '127.0.0.1', '-q', '42126'],
                           env=env, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node('w11r1_fake_lifecycle')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    processes = []

    def start(name):
        output = (tmp_path/'manager.log').open('w')
        proc = subprocess.Popen(['ros2', 'run', 't1_navigation', 'bounded_lifecycle_manager',
             '--ros-args', '-r', '__node:=w11r1_manager', '-p', f'node_names:=[{name}]',
             '-p', 'autostart:=false'], env=env, stdout=output, stderr=subprocess.STDOUT,
             start_new_session=True)
        processes.append((proc, output))
        client = node.create_client(ManageLifecycleNodes, '/w11r1_manager/manage_nodes')
        assert client.wait_for_service(timeout_sec=10.)
        return client

    start.processes = processes
    yield node, start
    for proc, output in processes:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=10.)
        output.close()
    executor.shutdown(timeout_sec=5.)
    thread.join(timeout=5.)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    dds.send_signal(signal.SIGINT)
    dds.wait(timeout=5.)
    log.close()
    for k, v in previous.items():
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = v
    assert all(proc.returncode == 0 for proc, _ in processes)


def response(future, seconds=25.):
    end = time.monotonic()+seconds
    while not future.done() and time.monotonic() < end:
        time.sleep(.02)
    assert future.done(), 'manager did not return a bounded failure'
    return future.result()


def test_lost_transition_ack_is_reconciled_without_replaying_configure(manager):
    node, start = manager
    state = {'value': 1, 'calls': []}
    group = ReentrantCallbackGroup()

    def get(req, res):
        res.current_state.id = state['value']
        return res

    def change(req, res):
        state['calls'].append(req.transition.id)
        if req.transition.id == 1:
            state['value'] = 2  # mutation applied, response deliberately arrives late
            time.sleep(2.3)
            res.success = True
        else:
            res.success = False  # stop before bonds; no fake success
        return res

    node.create_service(GetState, '/w11r1_stub/get_state', get, callback_group=group)
    node.create_service(ChangeState, '/w11r1_stub/change_state', change, callback_group=group)
    client = start('w11r1_stub')
    res = response(client.call_async(ManageLifecycleNodes.Request(command=0)))
    assert not res.success
    assert state['calls'] == [1, 3]
    # A failed epoch must not automatically retry/re-arm on a repeated STARTUP.
    assert not response(client.call_async(ManageLifecycleNodes.Request(command=0))).success
    assert state['calls'] == [1, 3]


def test_missing_lifecycle_service_returns_failure_instead_of_hanging(manager):
    node, start = manager
    client = start('w11r1_absent')
    begin = time.monotonic()
    assert not response(client.call_async(ManageLifecycleNodes.Request(command=0))).success
    assert time.monotonic()-begin < 15.


def test_sigint_with_formed_bond_exits_cleanly(manager):
    """Exercise actual bond handshake and OS shutdown, not an unactivated stub."""
    node, start = manager
    state = {'value': 1, 'active': True}

    def get(req, res):
        res.current_state.id = state['value']
        return res

    def change(req, res):
        state['value'] = {1: 2, 3: 3, 4: 2}[req.transition.id]
        res.success = True
        return res

    node.create_service(GetState, '/w11r1_stub/get_state', get)
    node.create_service(ChangeState, '/w11r1_stub/change_state', change)
    publisher = node.create_publisher(Status, '/bond', 10)

    def heartbeat():
        publisher.publish(Status(header=Header(
            stamp=node.get_clock().now().to_msg()), id='w11r1_stub',
            instance_id='w11r1_test_peer', active=state['active'],
            heartbeat_timeout=4., heartbeat_period=.1))

    def peer(msg):
        if msg.id == 'w11r1_stub' and msg.instance_id != 'w11r1_test_peer' and not msg.active:
            state['active'] = False
            heartbeat()

    node.create_subscription(Status, '/bond', peer, 10)
    node.create_timer(.05, heartbeat)
    client = start('w11r1_stub')
    assert response(client.call_async(ManageLifecycleNodes.Request(command=0))).success
    proc, _ = start.processes[-1]
    os.killpg(proc.pid, signal.SIGINT)
    assert proc.wait(timeout=5.) == 0

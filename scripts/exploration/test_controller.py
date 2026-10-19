"""Waiting must not bypass the mission deadline or unrelated safety latches."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec=importlib.util.spec_from_file_location('w16_controller',Path(__file__).with_name('controller.py'))
controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)


def waiting_node():
    return SimpleNamespace(interrupted=False,clock_value=5.,clock_wall=100.,
        started_sim=0.,limit=600.,started_wall=100.,sim=lambda:5.,
        observation_wait=True,gate=dict(state='STOPPED',reason='EXPLICIT_STOP_OR_CANCEL',health_reason=''))


def test_wait_recognizes_own_stop_but_rejects_other_latches(monkeypatch):
    monkeypatch.setattr(controller.rclpy,'spin_once',lambda *a,**kw:None)
    monkeypatch.setattr(controller.time,'monotonic',lambda:100.)
    node=waiting_node();controller.Explorer.spin(node)
    for changed in (dict(observation_wait=False),dict(gate=dict(state='FAULT',reason='EXPLICIT_STOP_OR_CANCEL')),
                    dict(gate=dict(state='STOPPED',reason='SENSOR_STALE')),
                    dict(gate=dict(state='STOPPED',reason='EXPLICIT_STOP_OR_CANCEL',health_reason='SCAN_STALE'))):
        node=waiting_node();node.__dict__.update(changed)
        with pytest.raises(RuntimeError,match='safety latch'):controller.Explorer.spin(node)


def test_wait_still_enforces_simulation_and_wall_deadlines(monkeypatch):
    monkeypatch.setattr(controller.rclpy,'spin_once',lambda *a,**kw:None)
    monkeypatch.setattr(controller.time,'monotonic',lambda:100.)
    node=waiting_node();node.sim=lambda:601.
    with pytest.raises(TimeoutError,match='simulation-time'):controller.Explorer.spin(node)
    node=waiting_node();node.started_wall=-2000.
    with pytest.raises(TimeoutError,match='wall-time'):controller.Explorer.spin(node)


def test_failed_arm_does_not_resume_waiting_navigation():
    node=waiting_node()
    reply=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(success=False,message='not healthy'))
    node.arm=SimpleNamespace(service_is_ready=lambda:True,call_async=lambda request:reply)
    node.wait=lambda predicate:predicate()
    with pytest.raises(RuntimeError,match='not healthy'):controller.Explorer.arm_navigation(node)
    assert node.observation_wait and node.gate['state']=='STOPPED'

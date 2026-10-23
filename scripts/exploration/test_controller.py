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


def blocked_mission(open_at, limit=600.):
    """Run the mission loop against observed blockage, without ROS or scene truth."""
    n=SimpleNamespace(now=1., grid=True, costmap=True, task=dict(offset=[1.,0.]),
        result={},gate=dict(state='ARMED'),observation_wait=False,nav_handle=None,
        visited=[],failed=[],sent=False,arms=0,events=[])
    n.sim=lambda:n.now
    n.pose=lambda *args:(1.,0.,0.) if n.sent else (0.,0.,0.)
    n.target=lambda:[1.,0.]
    n.navigation=n.planner=SimpleNamespace(server_is_ready=lambda:True)
    n.observed=lambda:SimpleNamespace(candidates=lambda *args:[],reobservation_candidates=lambda *args:[])
    n.snapshot=lambda observed:'observed-grid'
    n.plan=lambda *args:[(0.,0.),(1.,0.)] if n.now>=open_at else None
    n.emit=lambda event,**fields:n.events.append(dict(event=event,**fields))
    def spin(*args):
        n.now+=.1
        if n.now-1.>limit:raise TimeoutError('mission deadline')
    def wait(predicate,*args):
        while not predicate():spin()
    def arm():
        n.arms+=1;n.observation_wait=False;n.gate['state']='ARMED'
    def stop():n.gate['state']='STOPPED'
    def send(*args):
        assert n.gate['state']=='ARMED';n.sent=True
        return SimpleNamespace(done=lambda:False)
    n.spin,n.wait,n.arm_navigation,n.latch_stop,n.send=spin,wait,arm,stop,send
    n.verify_arrival=stop
    return n


def test_observed_blockage_can_clear_after_twenty_seconds_without_using_goal_budget():
    n=blocked_mission(open_at=36.)
    controller.Explorer.run(n)
    assert n.result['status']=='ARRIVED' and n.arms==2 and n.gate['state']=='STOPPED'
    decisions=[e for e in n.events if e['event']=='decision']
    assert len(decisions)==1 and decisions[0]['iteration']==0
    assert any(e['event']=='observation_wait_complete' for e in n.events)


def test_permanent_blockage_remains_bounded_and_never_rearms():
    n=blocked_mission(open_at=float('inf'))
    with pytest.raises(RuntimeError,match='bounded observation wait'):
        controller.Explorer.run(n)
    assert 61.<=n.now<64. and n.arms==1 and not n.sent and n.gate['state']=='STOPPED'


def test_short_global_budget_still_interrupts_longer_observation_wait():
    n=blocked_mission(open_at=36.,limit=20.)
    with pytest.raises(TimeoutError,match='mission deadline'):
        controller.Explorer.run(n)
    assert not n.sent and n.gate['state']=='STOPPED' and n.arms==1


def test_stationary_blockage_uses_a_validated_known_free_viewpoint():
    class Dispatched(Exception):pass
    n=blocked_mission(open_at=float('inf'))
    target=[1.,2.]
    n.observed=lambda:SimpleNamespace(candidates=lambda *args:[],
        reobservation_candidates=lambda *args:[dict(point=target,reason='known_free_reobservation')])
    n.plan=lambda point,observed:[(0.,0.),tuple(target)] if point==target else None
    def send(point,path,final):
        assert point==target and not final and n.gate['state']=='ARMED'
        assert path[-1]==tuple(target)
        raise Dispatched()
    n.send=send
    with pytest.raises(Dispatched):controller.Explorer.run(n)
    assert n.now>=21. and n.arms==2
    assert n.events[-1]['reason']=='known_free_reobservation' and n.events[-1]['iteration']==0

from pathlib import Path
from types import SimpleNamespace
import threading
import numpy as np
import pytest
from t1_mujoco.events import SceneEvents,load_scenario
from t1_mujoco.simulation import parameter_defaults,validate_parameters
from t1_mujoco.scenes import load_layout

WS=Path(__file__).resolve().parents[2]

def test_no_prior_map_and_required_safety():
    p=dict(parameter_defaults(),mode='advanced_mapping',scene='advanced_world',slam_params_file='/tmp/slam.yaml',navigation_params_file='/tmp/nav.yaml')
    validate_parameters(p)
    for updates in ({'map_file':'/tmp/old.yaml'},{'slam_params_file':''},{'safety_required':False}):
        with pytest.raises(ValueError):validate_parameters(dict(p,**updates))

def test_obstacle_requires_explicit_phase_enable_and_never_moves_base():
    scenario,_=load_scenario(WS/'src/t1_bringup/config/advanced_events.yaml','ADV01')
    data=SimpleNamespace(qpos=np.array([12.,8.,.7,1,0,0,0]),time=0.,mocap_pos=np.zeros((1,3)),mocap_quat=np.zeros((1,4)))
    owner=threading.get_ident()
    def check():assert threading.get_ident()==owner
    model=SimpleNamespace(body=lambda _:SimpleNamespace(id=0),body_mocapid=[0],geom=lambda _:SimpleNamespace(size=np.array([.25,.25,.75])))
    runner=SimpleNamespace(mj_model=model,mj_data=data,episode=0,state='READY',incoming=(.4,0,0),assert_owner=check)
    event=SceneEvents(runner,scenario);base=data.qpos.copy()
    for t in (1.,50.,400.):data.time=t;event.update()
    assert event.triggered_at is None
    event.enable_requested.set();event.update()
    assert event.triggered_at==400.
    data.time=403.;event.update()
    assert event.phase=='move_0'
    np.testing.assert_array_equal(data.qpos,base)
    runner.episode=1;event.update()
    assert event.triggered_at is None and not event.enable_requested.is_set()

def test_b_occluded_and_north_gallery_preserves_safety_margin():
    scene=load_layout(WS/'src/t1_mujoco/worlds/advanced_world.yaml')
    wall=scene['boxes'][0]
    assert wall['center'][0]-wall['size'][0]/2>0
    assert wall['center'][0]+wall['size'][0]/2<14
    assert wall['center'][1]-wall['size'][1]/2<0<wall['center'][1]+wall['size'][1]/2
    assert scene['bounds'][3]-(wall['center'][1]+wall['size'][1]/2)==7.5

def test_response_requires_perception_progress_and_arrival():
    import sys
    sys.path.insert(0,str(WS/'scripts/advanced'))
    from verification import response_verdict
    d=dict(active_scan_hit_frames=3,active_costmap_marked_frames=2,protective_stationary_s=0.,triggered=True,finished=True,replan={'replanned_while_active':True})
    assert response_verdict(d,True)=={'verdict':'PASS','response':'REPLAN_AND_CONTINUE'}
    for change in ({'active_scan_hit_frames':0},{'active_costmap_marked_frames':0},{'triggered':False},{'finished':False},{'replan':{'replanned_while_active':False}}):
        assert response_verdict(dict(d,**change),True)['verdict']=='FAIL'
    assert response_verdict(d,False)['verdict']=='FAIL'
    d.update(protective_stationary_s=.6,replan={'replanned_while_active':False})
    assert response_verdict(d,True)['response']=='PROTECTIVE_WAIT_AND_CONTINUE'

def test_replan_cannot_be_proved_by_a_new_plan_alone():
    import sys
    sys.path.insert(0,str(WS/'scripts/advanced'))
    from verification import replan_evidence
    obs=[{'phase':'move_0','sim_time':10.},{'phase':'hold_0','sim_time':12.}]
    plans=[dict(sim=9.,points=[[0,0],[5,0]]),dict(sim=11.,points=[[1,0],[2,1],[5,0]])]
    rows=[dict(sim=10.,x=1.,y=0.,mx=1.,my=0.),dict(sim=12.,x=2.,y=0.,mx=2.,my=0.)]
    assert not replan_evidence(rows,plans,obs,0.)['replanned_while_active']
    rows[-1].update(y=.5,my=.5)
    assert replan_evidence(rows,plans,obs,0.)['replanned_while_active']

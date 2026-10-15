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

def test_response_requires_visible_wait_resume_perception_and_arrival():
    import sys
    sys.path.insert(0,str(WS/'scripts/advanced'))
    from verification import response_verdict
    d=dict(active_scan_hit_frames=3,active_costmap_marked_frames=2,triggered=True,finished=True,
           blocked_wait=dict(stationary_s=2.1,resumed_m=2.))
    assert response_verdict(d,True)['verdict']=='PASS'
    for change in ({'active_scan_hit_frames':2},{'active_costmap_marked_frames':1},{'triggered':False},
                   {'finished':False},{'blocked_wait':dict(stationary_s=1.9,resumed_m=2.)},
                   {'blocked_wait':dict(stationary_s=3.,resumed_m=.9)}):
        assert response_verdict(dict(d,**change),True)['verdict']=='FAIL'
    assert response_verdict(d,False)['verdict']=='FAIL'


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


def test_ab_annotations_are_viewer_only_and_do_not_change_physics_or_rays():
    import mujoco
    from t1_mujoco.advanced_presentation import annotate
    model=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="box" pos="3 0 1" size=".5 .5 1"/></worldbody></mujoco>')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    positions=model.geom_pos.copy();sizes=model.geom_size.copy();qpos=data.qpos.copy()
    def ray():
        hit=np.array([-1],dtype=np.int32)
        return mujoco.mj_ray(model,data,np.array([0.,0.,1.]),np.array([1.,0.,0.]),None,1,-1,hit),hit[0]
    before=ray();scene=mujoco.MjvScene(model,maxgeom=8)
    viewer=SimpleNamespace(user_scn=scene,cam=SimpleNamespace())
    annotate(viewer)
    assert scene.ngeom==4 and model.ngeom==1 and ray()==before
    np.testing.assert_array_equal(model.geom_pos,positions)
    np.testing.assert_array_equal(model.geom_size,sizes)
    np.testing.assert_array_equal(data.qpos,qpos)

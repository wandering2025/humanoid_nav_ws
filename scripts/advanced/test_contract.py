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

def test_blockage_wait_requires_zero_command_actual_stillness_and_later_motion():
    import sys
    sys.path.insert(0,str(WS/'scripts/advanced'))
    from verification import blocked_wait_evidence
    rows=[]
    for i in range(101):
        t=i/10
        x=t if t<2 else (2. if t<=6 else 2.+.5*(t-6))
        rows.append(dict(sim=t,x=x,y=0.,yaw=0.,final=(0.,0.) if 2<=t<=6 else (.5,0.)))
    obs=[dict(phase='move_0',sim_time=1.),dict(phase='hold_0',sim_time=6.)]
    good=blocked_wait_evidence(rows,obs,(10.,0.,0.))
    assert good['stationary_s']>=3 and good['resumed_m']>1.5
    assert blocked_wait_evidence([dict(r,final=(.1,0.)) for r in rows],obs,(10.,0.,0.))['stationary_s']==0
    assert blocked_wait_evidence([dict(r,yaw=r['sim']) for r in rows],obs,(10.,0.,0.))['stationary_s']==0
    assert blocked_wait_evidence(rows,obs,(2.,0.,0.))['stationary_s']==0
    assert blocked_wait_evidence(rows,[],(10.,0.,0.))['stationary_s']==0


def test_goal_and_display_agree_and_cart_keeps_static_clearance():
    import yaml,mujoco
    from t1_mujoco.advanced_presentation import annotate
    route=yaml.safe_load((WS/'src/t1_bringup/config/advanced_route.yaml').read_text())
    layout=load_layout(WS/'src/t1_mujoco/worlds/advanced_world.yaml')
    from t1_mujoco.advanced_goal import load_goal
    goal=load_goal(WS/'presentation/goal.yaml')['goal']
    assert 'goal' not in route and 'navigation_approach' not in route
    x,y,_=goal;xmin,xmax,ymin,ymax=layout['bounds']
    assert min(x-xmin,xmax-x,y-ymin,ymax-y)>=2.25
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
    scn=mujoco.MjvScene(m,maxgeom=8);annotate(SimpleNamespace(user_scn=scn,cam=SimpleNamespace()))
    np.testing.assert_allclose(scn.geoms[2].pos[:2],goal[:2])
    scenario,_=load_scenario(WS/'src/t1_bringup/config/advanced_events.yaml','ADV01')
    size=layout['obstacle_slots'][0]['size']
    for cx,cy in [scenario['start']]+[leg['to'] for leg in scenario['legs']]:
        assert xmin<cx-size[0]/2<cx+size[0]/2<xmax
        assert ymin<cy-size[1]/2<cy+size[1]/2<ymax


def test_delayed_overview_never_masquerades_as_execution_plan():
    # Execute the actual callback without constructing ROS processes or GPU owners.
    import ast,copy
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import serialize_message
    tree=ast.parse((WS/'scripts/advanced/demo.py').read_text())
    geometry=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='path_geometry')
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Advanced')
    callback=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='live_plan')
    ns={};exec(compile(ast.Module(body=[geometry,callback],type_ignores=[]),'actual_display_callback','exec'),ns)
    overview=NavPath();overview.header.frame_id='map'
    for x,y in ((0.,0.),(3.,8.),(12.,0.)):
        p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;overview.poses.append(p)
    published=[];owner=SimpleNamespace(computing_overview=True,plan_viz=SimpleNamespace(publish=published.append),node=SimpleNamespace(sim=100.),active_target=(13.4,6.5,0.))
    cb=ns['live_plan'];cb(owner,overview);assert not published
    owner.overview_geometry=ns['path_geometry'](overview);owner.computing_overview=False
    late=copy.deepcopy(overview);late.header.stamp.sec=101;late.poses[0].header.stamp.sec=101
    cb(owner,late);assert not published and owner.active_target==(13.4,6.5,0.)
    actual=copy.deepcopy(late);actual.poses[0].pose.position.x=13.;actual.poses[0].pose.position.y=6.5
    before=serialize_message(actual);cb(owner,actual)
    assert len(published)==1 and serialize_message(published[0])==before
    assert owner.live_plan_sim==100. and owner.active_target==(12.,0.,0.)

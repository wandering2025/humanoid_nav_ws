import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from evaluation.metrics import body_clearance, classify, stopping_metrics


def test_signed_geometry_and_moving_obstacle(tmp_path):
    xml=tmp_path/'fixture.xml'
    xml.write_text('''<mujoco><worldbody><geom name="ground" type="plane" size="5 5 .1"/>
      <body name="trunk" pos="0 0 1"><freejoint/><geom name="robot" type="sphere" size=".2"/>
      <geom name="visual_only" type="sphere" size="10" contype="0" conaffinity="0"/></body>
      <body name="blocker" mocap="true" pos="2 0 1"><geom name="obstacle" type="box" size=".5 .5 .5"/></body>
      </worldbody></mujoco>''')
    states=tmp_path/'states.jsonl'
    states.write_text('\n'.join(json.dumps(dict(sim_time=t,qpos=[0,0,1,1,0,0,0],mocap_pos=[[x,0,1]],mocap_quat=[[1,0,0,0]])) for t,x in [(0,2),(.1,.6)]))
    result=body_clearance({'scene_file':str(xml)},states)
    assert result['min_m']==pytest.approx(-.1,abs=1e-8)
    assert result['sim_time']==.1 and result['robot_geoms']==1 and result['environment_geoms']==1


@pytest.mark.parametrize('result,valid,outcome',[
    ({'outcome':'SUCCESS','metrics':{}},True,'SUCCESS'),
    ({'outcome':'SUCCESS','metrics':{}},False,'INVALID'),
    ({'metrics':{'env_contacts':{'obstacle':1}}},True,'FAIL_COLLISION'),
    ({'metrics':{'env_contacts':{'ground_nonfoot':1}}},True,'FAIL_FALL'),
    ({'error':'launch failed'},True,'FAIL_STARTUP'),
    ({'nav_active_sim':12,'nav2':{'timed_out':True},'metrics':{}},True,'FAIL_TIMEOUT'),
    ({'nav_active_sim':12,'metrics':{'nav2_status':'ABORTED'}},True,'FAIL_PLANNING'),
])
def test_failure_precedence(result,valid,outcome):
    assert classify(result,valid)==outcome


def test_stop_displacement_includes_rebound():
    t=np.arange(200)*.02
    x=np.minimum(t*.4,1.36)
    trace=dict(sim_time=t,x_w=x,y_w=np.zeros(200),yaw_w=np.zeros(200),u_pol_x=np.where(t<3.2,.5,0),
               u_pol_y=np.zeros(200),u_pol_wz=np.zeros(200),v_meas_x=np.where(t<3.4,.4,0))
    result=stopping_metrics([],trace,[dict(kind='collision_monitor',action_type=1,sim=3.2)])
    assert len(result)==1
    assert result[0]['max_displacement_after_zero_m']==pytest.approx(.08)
    assert result[0]['resumed_sim'] is None


def test_frozen_suite_coverage_and_safe_approach():
    ws=Path(__file__).resolve().parents[3]
    suite=yaml.safe_load((ws/'src/t1_bringup/config/test_suite_v1.yaml').read_text())
    events=yaml.safe_load((ws/suite['events_file']).read_text())['scenarios']
    cases=suite['cases']
    assert len(cases)==40 and len({c['id'] for c in cases})==40
    for label,first in [('static',1000),('crossing',2000),('blocking',3000),('sudden',4000)]:
        rows=[c for c in cases if c['suite']==label]
        assert len(rows)==10 and [c['seed'] for c in rows]==list(range(first,first+10))
        for c in rows:
            if label!='static':
                e=events[c['scenario']]
                assert e['seed']==c['seed']
                if label=='sudden':
                    assert e['initial_surface_gap_lower_bound_m'] > 1.65+.245+.61*.65+.2*.45
                if label=='blocking':
                    assert e['legs'][0]['hold']>suite['criteria']['time_limit_s']

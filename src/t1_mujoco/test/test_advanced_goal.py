"""Meaningful operator-boundary and shared-goal integration contracts, CPU only."""
import hashlib
import json
from types import SimpleNamespace
import numpy as np
import pytest
from t1_mujoco.advanced_goal import load_goal, parse_goal, runtime_goal, SNAPSHOT_ENV


def text(x=12., y=0.):
    return f'schema: 1\nframe: map\ngoal: {{x: {x}, y: {y}}}\n'


@pytest.mark.parametrize('x,y', [(11.,-.5),(11.,.5),(12.,-.5),(12.,.5),(11.5,-.25)])
def test_supported_rectangle_keeps_entire_approach_clear(x,y):
    p=parse_goal(text(x,y),'/tmp/goal.yaml')
    assert p['goal'][:2]==[x,y] and p['navigation_approach'][:2]==[x,y+2.5]
    assert p['selection_min_clearance_m']>=2.4-1e-9


@pytest.mark.parametrize('content', [text(10.99,0),text(12.01,0),text(12,-.51),text(12,.51),
    text('.nan',0),text('.inf',0),text('true',0),text('"12"',0),
    'schema: 1\nframe: odom\ngoal: {x: 12, y: 0}\n',
    'schema: 1\nframe: map\ngoal: {x: 12, y: 0, yaw: 0}\n',
    'schema: 1\nframe: map\ngoal: {x: 11, x: 12, y: 0}\n'])
def test_invalid_selection_never_silently_falls_back(content):
    with pytest.raises(ValueError):parse_goal(content,'/tmp/goal.yaml')


def test_each_launch_reads_new_file_but_inflight_snapshot_stays_consistent(tmp_path,monkeypatch):
    source=tmp_path/'goal.yaml';source.write_bytes(text().replace('\n','\r\n').encode())
    first=load_goal(source)
    assert first['source_sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
    snapshot=tmp_path/'snapshot.json';snapshot.write_text(json.dumps(first));monkeypatch.setenv(SNAPSHOT_ENV,str(snapshot))
    source.write_text(text(11,.5));second=load_goal(source)
    assert first['goal']!=second['goal'] and first['source_sha256']!=second['source_sha256']
    assert runtime_goal()==first
    bad=dict(first,goal=second['goal']);snapshot.write_text(json.dumps(bad))
    with pytest.raises(ValueError,match='inconsistent'):runtime_goal()


def test_changed_b_marker_is_render_only_and_matches_actual_snapshot(tmp_path,monkeypatch):
    import mujoco
    from t1_mujoco.advanced_presentation import annotate
    p=parse_goal(text(11,.5),str(tmp_path/'goal.yaml'));snapshot=tmp_path/'snapshot.json';snapshot.write_text(json.dumps(p));monkeypatch.setenv(SNAPSHOT_ENV,str(snapshot))
    m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="box" pos="3 0 1" size=".5 .5 1"/></worldbody></mujoco>')
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    pos=m.geom_pos.copy();size=m.geom_size.copy();q=d.qpos.copy()
    def ray():return mujoco.mj_ray(m,d,np.array([0.,0.,1.]),np.array([1.,0.,0.]),None,1,-1,np.array([-1],dtype=np.int32))
    before=ray();v=SimpleNamespace(user_scn=mujoco.MjvScene(m,maxgeom=8),cam=SimpleNamespace())
    annotate(v)
    np.testing.assert_allclose(v.user_scn.geoms[2].pos[:2],p['goal'][:2])
    np.testing.assert_array_equal(m.geom_pos,pos);np.testing.assert_array_equal(m.geom_size,size);np.testing.assert_array_equal(d.qpos,q)
    assert ray()==before and m.ngeom==1

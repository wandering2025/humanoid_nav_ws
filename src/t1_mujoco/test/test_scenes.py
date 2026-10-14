"""Reject dangerous/malformed scene configs before physics construction."""
from pathlib import Path
import math
import yaml
import pytest
from t1_mujoco.scenes import load_layout, layout_path

LAYOUT=Path(__file__).parents[1]/'worlds/static_world.yaml'


@pytest.mark.parametrize('fault', ['nan_pose','reversed_bounds','negative_box','duplicate_box','bad_slot','endpoint_inside'])
def test_malformed_layout_rejected(tmp_path,fault):
    data=yaml.safe_load(LAYOUT.read_text())
    if fault=='nan_pose':data['initial_pose'][0]=math.nan
    elif fault=='reversed_bounds':data['bounds'][0]=10
    elif fault=='negative_box':data['boxes'][0]['size'][0]=-1
    elif fault=='duplicate_box':data['boxes'].append(data['boxes'][0].copy())
    elif fault=='bad_slot':data['obstacle_slots']=[dict(name='unsafe',region='missing',shape='box',size=[.5,.5,1.5],motion_implemented=True)]
    elif fault=='endpoint_inside':data['initial_pose']=[3,0,0]
    path=tmp_path/'scene.yaml';path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_layout(path)


def test_unknown_scene_cannot_silently_use_upstream(tmp_path):
    with pytest.raises(ValueError,match='known layout name'):layout_path('typo_world',tmp_path)

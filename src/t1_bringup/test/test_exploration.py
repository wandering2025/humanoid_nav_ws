import math
from pathlib import Path

import numpy as np
import pytest

from t1_bringup.exploration import ObservedMap, transform_point
from t1_mujoco.exploration_task import load_task


def room():
    data = np.zeros((120,160), dtype=np.int16)
    data[[0,-1],:] = 100; data[:,[0,-1]] = 100
    return data


def test_start_and_rotated_grid_coordinates():
    assert transform_point((2,1),(3,4,math.pi/2)) == pytest.approx((2,6))
    grid = ObservedMap(room(), .2, (3,4,math.pi/2), (0,10))
    for cell in ((20,30),(70,90)):
        assert grid.cell(grid.point(cell)) == cell
    assert grid.cell((3.01,3.99))[1] == -1  # floor, not truncation toward zero


def test_unknown_and_wall_goals_not_reachable_and_no_corner_shortcut():
    data = room(); data[:,80:] = -1
    data[35:65,35:65] = 100
    grid = ObservedMap(data,.2,(0,0,0),(5,5))
    assert grid.is_reachable((5,5))
    assert not grid.is_reachable((20,5))
    assert not grid.is_reachable((10,10))
    assert not grid.path_known([(5,5),(15,15)])
    assert not grid.path_known([(5,5),(31,5)])


def test_frontier_targets_are_safe_observed_and_respond_to_wall_layout():
    data = room(); data[:,65:] = -1
    data[75:,1:135] = 0; data[1:75,60:65] = 100
    grid = ObservedMap(data,.2,(0,0,0),(5,5))
    candidates = grid.candidates((5,5),(21,5))
    assert candidates and candidates[0]['point'][1] > 10
    assert all(grid.is_reachable(c['point']) and c['gain'] >= 1 for c in candidates)
    flipped = ObservedMap(np.flipud(data),.2,(0,0,0),(5,19))
    other = flipped.candidates((5,19),(21,19))
    assert other and other[0]['point'][1] < 14
    # Failure exclusion and revisit cost prevent repeatedly selecting a dead end.
    failed = grid.candidates((5,5),(21,5),failed=[candidates[0]['point']])
    assert all(math.dist(c['point'], candidates[0]['point']) >= 1.5 for c in failed)
    revisited = grid.candidates((5,5),(21,5),visited=[candidates[0]['point']]*3)
    assert math.dist(revisited[0]['point'],candidates[0]['point']) > 1.5


def test_enclosed_room_and_too_narrow_door_do_not_create_a_route():
    data = room(); data[:,80:] = -1; data[:,79] = 100
    grid = ObservedMap(data,.2,(0,0,0),(5,5))
    assert not grid.candidates((5,5),(25,5))
    data = room(); data[:,80] = 100; data[50:60,80] = 0
    grid = ObservedMap(data,.2,(0,0,0),(5,5))
    assert not grid.is_reachable((25,5))


def test_costmap_costs_are_not_raw_occupancy_probabilities():
    data = room(); data[30:90,30:90] = 80
    grid = ObservedMap(data,.2,(0,0,0),(10,10),inflated=True)
    assert grid.is_reachable((10,10))  # inflation cost 80 is traversable
    data[:,80] = 99
    grid = ObservedMap(data,.2,(0,0,0),(10,10),inflated=True)
    assert not grid.is_reachable((25,10))  # inscribed 99 remains blocked


def test_input_contains_only_relative_point_and_does_not_read_scene(tmp_path, monkeypatch):
    task = tmp_path/'task.yaml'
    task.write_text('schema: 1\nframe: start\ngoal: {x: -42, y: 30}\n')
    original = Path.read_text
    def only_task(path, *args, **kwargs):
        assert path == task
        return original(path,*args,**kwargs)
    monkeypatch.setattr(Path,'read_text',only_task)
    assert load_task(task)['offset'] == [-42.,30.]
    for text in ('schema: 1\nframe: map\ngoal: {x: 1,y: 2}',
                 'schema: 1\nframe: start\ngoal: {x: .nan, y: 0}',
                 'schema: 1\nframe: start\ngoal: {x: 1, x: 2, y: 3}',
                 'schema: 1\nframe: start\ngoal: {x: 1, y: 2, yaw: 0}'):
        task.write_text(text)
        with pytest.raises(ValueError):
            load_task(task)

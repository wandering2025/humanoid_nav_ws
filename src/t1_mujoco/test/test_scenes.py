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
    elif fault=='endpoint_inside':data['initial_pose']=[*data['boxes'][0]['center'][:2],0]
    path=tmp_path/'scene.yaml';path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):load_layout(path)


def test_unknown_scene_cannot_silently_use_upstream(tmp_path):
    with pytest.raises(ValueError,match='known layout name'):layout_path('typo_world',tmp_path)


def _clearance(data, x, y):
    xmin, xmax, ymin, ymax = data['bounds']
    values = [x-xmin, xmax-x, y-ymin, ymax-y]
    for box in data['boxes']:
        (cx, cy, _), (sx, sy, _) = box['center'], box['size']
        values.append(math.hypot(max(abs(x-cx)-sx/2, 0), max(abs(y-cy)-sy/2, 0)))
    return min(values)


def test_static_world_respects_frozen_w10_zones():
    """ADR-015: A/B allow in-place turns outside the 1.95 m slowdown circle and
    the straight A-B segment is blocked, so W11 must detour autonomously."""
    data = load_layout(LAYOUT)
    for x, y, _ in (data['initial_pose'], data['goal']):
        assert _clearance(data, x, y) >= 2.25
    (ax, ay, _), (bx, by, _) = data['initial_pose'], data['goal']
    assert min(_clearance(data, ax+(bx-ax)*t/100, ay+(by-ay)*t/100) for t in range(101)) < 1.65
    for side in (1, -1):  # both passages at y=+-2.75 keep >=2.25 m to every surface
        assert min(_clearance(data, x/10, side*2.75) for x in range(15, 76)) >= 2.25


def test_start_pose_parsing_and_validation():
    from t1_mujoco.scenes import parse_start_pose, validate_layout
    assert parse_start_pose('') is None
    assert parse_start_pose('1.5 0 0.25') == [1.5, 0.0, 0.25]
    for bad in ('1 2', 'nan 0 0', '1 2 3 4'):
        with pytest.raises(ValueError):
            parse_start_pose(bad)
    data = load_layout(LAYOUT)
    island = data['boxes'][0]['center']
    with pytest.raises(ValueError, match='inside obstacle'):
        validate_layout(dict(data, initial_pose=[island[0], island[1], 0.0]))
    with pytest.raises(ValueError, match='outside inner walls'):
        validate_layout(dict(data, initial_pose=[50.0, 0.0, 0.0]))


def test_contact_events_dedupe_and_gap(tmp_path):
    """Event logic only; the classification runs on real MjData in W11 evidence."""
    import csv
    from t1_mujoco.contacts import ContactMonitor
    m = ContactMonitor.__new__(ContactMonitor)
    m.gap, m.open = .04, {}
    m.counts = dict(static_env=0, obstacle=0, ground_nonfoot=0)
    m.file = (tmp_path/'c.csv').open('w', newline='')
    m.writer = csv.DictWriter(m.file, fieldnames=('episode', 't_start', 't_end', 'robot_geom', 'robot_body', 'other_geom', 'other_body', 'other_class', 'max_penetration_m', 'n_substeps'))
    m.writer.writeheader()

    class Model:
        geom_bodyid = [0, 1, 2]
    m.model = Model()
    m.name = lambda kind, index: f'n{index}'
    for t in (1.0, 1.002, 1.03):          # one event: gaps < 0.04 s
        m.update(0, t, 1, 2, 'static_env', .001)
        m.flush(t)
    m.flush(1.2)
    m.update(0, 1.3, 1, 2, 'static_env', .002)   # new event after >= gap
    m.close()
    rows = list(csv.DictReader((tmp_path/'c.csv').open()))
    assert m.counts['static_env'] == 2 and len(rows) == 2
    assert rows[0]['n_substeps'] == '3' and float(rows[0]['t_end']) == 1.03

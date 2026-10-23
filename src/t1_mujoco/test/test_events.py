"""Event clock and safety boundaries; real mocap/rays/contacts tested separately."""
from pathlib import Path
from types import SimpleNamespace
import threading
import math

import numpy as np
import pytest

from t1_mujoco.events import SceneEvents, approach_ready, load_scenario
from t1_mujoco.simulation import parameter_defaults, validate_parameters

EVENTS = Path(__file__).parents[2]/'t1_bringup/config/w12_events.yaml'


def test_mapping_rejects_dynamic_events():
    p = dict(parameter_defaults(), mode='mapping', scene='dynamic_crossing_world',
             events_file=str(EVENTS), scenario='blocking')
    with pytest.raises(ValueError, match='mapping excludes bodies'):
        validate_parameters(p)


def test_interpolation_pause_reset_and_single_owner():
    scenario, provenance = load_scenario(EVENTS, 'approach')
    assert len(provenance['sha256']) == 64
    model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0), body_mocapid=[0],
                            geom=lambda _: SimpleNamespace(size=np.array([.25, .25, .75])))
    data = SimpleNamespace(qpos=np.zeros(7), time=0., mocap_pos=np.zeros((1, 3)),
                           mocap_quat=np.zeros((1, 4)))
    owner = threading.get_ident()
    def check():
        if threading.get_ident() != owner:
            raise RuntimeError('outside owner')
    runner = SimpleNamespace(mj_model=model, mj_data=data, episode=0, state='READY',
                             incoming=(.45, 0, 0), assert_owner=check)
    events = SceneEvents(runner, scenario)
    qpos = data.qpos.copy()
    events.update()
    assert events.triggered_at is None
    data.qpos[0] = 3.21
    data.time = 5
    events.update()
    assert events.triggered_at == 5
    data.time = 7
    events.update()
    np.testing.assert_allclose(events.position, [6, 1.8])
    for _ in range(20):  # wall-clock work with frozen simulation time
        events.update()
    np.testing.assert_allclose(events.position, [6, 1.8])
    data.time = 10.1
    events.update()
    assert events.phase == 'hold_0'
    data.time = 14
    events.update()
    np.testing.assert_allclose(events.position, [6, .6])
    runner.episode = 1
    runner.state = 'STARTING'
    events.update()
    assert events.triggered_at is None
    np.testing.assert_allclose(events.position, scenario['start'])
    np.testing.assert_allclose(data.qpos[1:], qpos[1:])
    errors = []
    def off_thread():
        try:
            events.update()
        except RuntimeError as exc:
            errors.append(str(exc))
    thread = threading.Thread(target=off_thread)
    thread.start(); thread.join()
    assert errors == ['outside owner']


def test_reject_invalid_event_speed(tmp_path):
    text = EVENTS.read_text().replace('speed: 0.6', 'speed: 8.0')
    path = tmp_path/'events.yaml'; path.write_text(text)
    with pytest.raises(ValueError, match='event speed'):
        load_scenario(path, 'approach')


def test_exploration_approach_requires_nearby_southward_motion():
    path = EVENTS.with_name('exploration_events.yaml')
    scenario, _ = load_scenario(path, 'EXP01')
    def pose(y, yaw):
        return np.array([11., y, .67, math.cos(yaw/2), 0., 0., math.sin(yaw/2)])
    # Original failure: crossed x threshold while still exploring eastward.
    assert not approach_ready(scenario, pose(7.66, -.44))
    assert not approach_ready(scenario, pose(6.1, -.44))
    assert not approach_ready(scenario, pose(6.1, 2.8))
    assert not approach_ready(scenario, pose(8., -math.pi/2))
    assert not approach_ready(scenario, pose(4., -math.pi/2))
    # A measured late-turn exploration approach starts forward at y=5.571.
    assert approach_ready(scenario, pose(5.570923273146264, -1.7766507103206945))
    assert not approach_ready(scenario, pose(4.99, -math.pi/2))
    for y in (5.0, 5.6, 6.5, 7.4):
        assert approach_ready(scenario, pose(y, -math.pi/2))
        assert approach_ready(scenario, pose(y, 3*math.pi/2))
    legacy, _ = load_scenario(EVENTS, 'approach')
    assert approach_ready(legacy, pose(7.66, -.44))


@pytest.mark.parametrize('gate', [False, {}, {'y_min': 0},
    dict(y_min=7.4, y_max=5.6, heading=0., heading_tolerance=.6),
    dict(y_min=5.6, y_max=7.4, heading=float('nan'), heading_tolerance=.6),
    dict(y_min=5.6, y_max=7.4, heading=0., heading_tolerance=True),
    dict(y_min=5.6, y_max=7.4, heading=0., heading_tolerance=0.)])
def test_reject_invalid_approach_conditions(tmp_path, gate):
    import yaml
    path = EVENTS.with_name('exploration_events.yaml')
    data = yaml.safe_load(path.read_text())
    data['scenarios']['EXP01']['trigger_approach'] = gate
    out = tmp_path/'events.yaml'; out.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='trigger_approach'):
        load_scenario(out, 'EXP01')

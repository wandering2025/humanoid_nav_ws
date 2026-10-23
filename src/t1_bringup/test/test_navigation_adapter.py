"""ADR-016 pre-monitor shaping: every output is zero or inside the W05 envelope."""
import itertools
import math
from pathlib import Path

import pytest
import yaml

from t1_bringup.navigation_adapter import load_envelope, shape

COMMAND_MAP = Path(__file__).resolve().parents[2]/'t1_mujoco/config/command_map.yaml'
SHAPING = dict(vx_lift_threshold=.01, vx_lift_target=.33, wz_lift_threshold=.2, wz_in_place_target=.6,
               min_walking_radius=.70)


@pytest.fixture(scope='module')
def env():
    return load_envelope(COMMAND_MAP)


def feasible(vx, wz, env):
    if vx == 0 and wz == 0:
        return True
    if vx == 0:
        return env['wz_in_place_min'] <= abs(wz) <= env['wz_in_place_max']
    return env['vx_min'] <= vx <= env['vx_max'] and abs(wz) <= env['wz_walking_max']


def test_envelope_matches_w05(env):
    assert (env['vx_min'], env['vx_max'], env['wz_walking_max']) == (.315, .75, .45)
    assert (env['wz_in_place_min'], env['wz_in_place_max']) == (.56, .85)


def test_dense_grid_output_is_feasible_and_sign_preserving(env):
    grid = [i/100 for i in range(-100, 101, 2)]
    for vx, wz in itertools.product(grid, grid):
        ox, owz, _ = shape(vx, wz, env, **SHAPING)
        assert feasible(ox, owz, env), (vx, wz, ox, owz)
        assert ox >= 0
        assert owz == 0 or math.copysign(1, owz) == math.copysign(1, wz)
        if vx >= env['vx_min']:  # inside the walking envelope nothing is raised
            assert ox <= vx and abs(owz) <= abs(wz)


@pytest.mark.parametrize('vx,wz,expected', [
    (.2, .1, (.33, .1)), (.005, .0, (0., 0.)), (.04, .0, (.33, 0.)), (.1, .0, (.33, 0.)), (.05, .3, (0., .6)), (0., -.7, (0., -.7)),
    (.09, .45, (0., .6)), (.2, -.3, (0., -.6)), (.25, .3, (.33, .3)), (.06, .1, (.33, .1)),
    (0., .1, (0., 0.)), (.5, .8, (.5, .45)), (-.3, 0., (0., 0.)), (0., 2., (0., .85)), (.9, 0., (.75, 0.))])
def test_examples(env, vx, wz, expected):
    assert shape(vx, wz, env, **SHAPING)[:2] == pytest.approx(expected)


def test_nonfinite_reaches_gate_unchanged(env):
    vx, wz, reason = shape(math.nan, 0., env, **SHAPING)
    assert math.isnan(vx) and reason == 'nonfinite_passthrough'


def test_uncalibrated_map_rejected(tmp_path):
    data = yaml.safe_load(COMMAND_MAP.read_text())
    data['calibrated'] = False
    path = tmp_path/'map.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='calibrated'):
        load_envelope(path)

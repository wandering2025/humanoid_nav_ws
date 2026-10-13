import copy
import itertools
import math
from pathlib import Path

import pytest
import yaml

from t1_mujoco.command_map import CommandMap, CommandMapError
from t1_mujoco.control import InputBuffer

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'command_map.yaml'

FIXTURE = dict(
    schema=1, calibrated=True, evidence={'report': 'x'}, applicability='test',
    capabilities=dict(forward=True, reverse_allowed=False, lateral_allowed=False, in_place_turn_allowed=True),
    envelope=dict(vx_min=0.3, vx_max=0.7, wz_walking_max=0.4, wz_in_place_min=0.45, wz_in_place_max=0.9),
    tables=dict(vx=[[0.28, 0.4], [0.42, 0.5], [0.75, 0.8]],
                wz_walking_pos=[[0.1, 0.1], [0.45, 0.5]], wz_walking_neg=[[0.09, 0.1], [0.42, 0.5]],
                wz_in_place_pos=[[0.43, 0.5], [0.99, 1.0]], wz_in_place_neg=[[0.34, 0.5], [0.97, 1.0]]))


def maps():
    yield CommandMap(copy.deepcopy(FIXTURE))
    if CONFIG.is_file():
        yield CommandMap.load(CONFIG)


GRID = [round(x, 3) for x in [i * 0.025 for i in range(-48, 49)]]


@pytest.mark.parametrize('m', list(maps()), ids=lambda m: m.source)
def test_zero_maps_to_zero_exactly(m):
    assert m.guard((0.0, 0.0, 0.0)) == ((0.0, 0.0, 0.0), [])
    assert m.expected((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('m', list(maps()), ids=lambda m: m.source)
def test_only_reduces_never_raises_and_keeps_sign(m):
    for vx, vy, wz in itertools.product(GRID, (0.0, 0.2, -0.2), GRID):
        u, _ = m.guard((vx, vy, wz))
        e = m.expected(u)
        assert all(math.isfinite(x) for x in u) and all(abs(x) <= 1.0 for x in u)
        assert u[1] == 0.0 and e[1] == 0.0  # lateral disabled
        for req, got in ((vx, e[0]), (wz, e[2])):
            assert abs(got) <= abs(req) + 1e-12            # |M^-1(u)| <= |v|
            assert got == 0.0 or math.copysign(1, got) == math.copysign(1, req)
        if vx <= 0:
            assert u[0] == 0.0  # reverse disabled, never raised


@pytest.mark.parametrize('m', list(maps()), ids=lambda m: m.source)
def test_deadband_zeroes_requests_below_walking_minimum(m):
    vmin = m.env['vx_min']
    for vx in (0.01, 0.1, vmin * 0.5, vmin - 1e-6):
        u, flags = m.guard((vx, 0.0, 0.0))
        assert u == (0.0, 0.0, 0.0) and 'vx_deadband' in flags
    u, _ = m.guard((vmin, 0.0, 0.0))
    assert u[0] > 0 and m.expected(u)[0] == pytest.approx(vmin)
    wmin = m.env['wz_in_place_min']
    u, flags = m.guard((0.0, 0.0, wmin * 0.9))
    assert u == (0.0, 0.0, 0.0) and 'wz_deadband' in flags


@pytest.mark.parametrize('m', list(maps()), ids=lambda m: m.source)
def test_monotone_per_axis(m):
    speeds = [m.env['vx_min'] + i * 0.01 for i in range(int((m.env['vx_max'] - m.env['vx_min']) / 0.01) + 1)]
    ux = [m.guard((v, 0.0, 0.0))[0][0] for v in speeds]
    assert all(b >= a for a, b in zip(ux, ux[1:]))
    for vx in (0.0, m.env['vx_min'] + 0.05):
        for sign in (1, -1):
            uw = [abs(m.guard((vx, 0.0, sign * w * 0.01))[0][2]) for w in range(0, 101)]
            assert all(b >= a for a, b in zip(uw, uw[1:]))


@pytest.mark.parametrize('m', list(maps()), ids=lambda m: m.source)
def test_envelope_clip(m):
    u, flags = m.guard((5.0, 0.0, 0.0))
    assert 'vx_clipped' in flags and m.expected(u)[0] == pytest.approx(m.env['vx_max'])
    u, flags = m.guard((m.env['vx_max'], 0.0, -5.0))
    assert 'wz_clipped' in flags and m.expected(u)[2] == pytest.approx(-m.env['wz_walking_max'])


@pytest.mark.parametrize('change', [
    ('schema', 2), ('calibrated', False), ('tables.vx', [[0.3, 0.5], [0.2, 0.6]]),
    ('tables.vx', [[0.0, 0.0], [0.4, 0.5]]), ('tables.vx', [[0.3, 0.5], [0.4, 1.2]]),
    ('envelope.vx_max', 2.0), ('envelope.vx_min', float('nan')),
    ('capabilities.reverse_allowed', True), ('tables.wz_in_place_neg', None)])
def test_invalid_maps_rejected(change):
    data = copy.deepcopy(FIXTURE)
    path, value = change
    target = data
    keys = path.split('.')
    for k in keys[:-1]:
        target = target[k]
    target[keys[-1]] = value
    with pytest.raises(CommandMapError):
        CommandMap(data)


def test_load_requires_absolute_existing_file(tmp_path):
    with pytest.raises(CommandMapError):
        CommandMap.load('relative.yaml')
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump(FIXTURE))
    assert CommandMap.load(path).env['vx_min'] == 0.3


def test_input_buffer_uses_map_only_when_given():
    m = CommandMap(copy.deepcopy(FIXTURE))
    b = InputBuffer()
    b.boundary('READY', 10.)
    b.receive((0.1, 0, 0, 0, 0, 0), wall=20.)  # below walking minimum
    assert b.consume(10., 20., .5, .5, False)[1:] == ((0., 0., 0.), 'UNCALIBRATED')
    assert b.consume(10., 20., .5, .5, False, m.guard)[1:] == ((0., 0., 0.), 'ACTIVE')
    b.receive((0.5, 0, 0, 0, 0, float('nan')), wall=20.)
    assert b.consume(10., 20., .5, .5, False, m.guard)[1:] == ((0., 0., 0.), 'INVALID')
    b.receive((0.5, 0, 0, 0, 0, 0), wall=20.)
    assert b.consume(10.6, 20., .5, .5, False, m.guard)[1:] == ((0., 0., 0.), 'TIMEOUT')
    assert b.consume(10., 20., .5, .5, False, m.guard)[1][0] > 0

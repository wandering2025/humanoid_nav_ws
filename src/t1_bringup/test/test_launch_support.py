import pytest
from t1_bringup.launch_support import validate_request


@pytest.mark.parametrize('mode', ['mapping', 'navigation', 'evaluation'])
def test_future_simulation_cannot_silently_start(mode):
    with pytest.raises(RuntimeError, match='W04 simulation supports sim_only'):
        validate_request(mode, 'true')


def test_unknown_mode():
    with pytest.raises(RuntimeError, match='Unknown mode'):
        validate_request('unknown', 'false')


def test_invalid_boolean():
    with pytest.raises(RuntimeError, match='enable_sim must be true or false'):
        validate_request('sim_only', 'maybe')


def test_sim_only_enabled():
    validate_request('sim_only', 'true')

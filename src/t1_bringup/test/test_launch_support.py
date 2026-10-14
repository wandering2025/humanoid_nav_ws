import pytest
from t1_bringup.launch_support import validate_request


@pytest.mark.parametrize('mode', ['navigation', 'evaluation'])
def test_future_simulation_cannot_silently_start(mode):
    with pytest.raises(RuntimeError, match='require W10/W11'):
        validate_request(mode, 'true')


def test_unknown_mode():
    with pytest.raises(RuntimeError, match='Unknown mode'):
        validate_request('unknown', 'false')


def test_invalid_boolean():
    with pytest.raises(RuntimeError, match='enable_sim must be true or false'):
        validate_request('sim_only', 'maybe')


def test_sim_only_enabled():
    validate_request('sim_only', 'true')


@pytest.mark.parametrize("mode", ["mapping", "localization"])
def test_w09_modes_enabled(mode):
    validate_request(mode, "true")

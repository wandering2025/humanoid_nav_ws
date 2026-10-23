import pytest
from t1_bringup.launch_support import validate_request


def test_evaluation_cannot_silently_start():
    with pytest.raises(RuntimeError, match='requires W13'):
        validate_request('evaluation', 'true')


def test_unknown_mode():
    with pytest.raises(RuntimeError, match='Unknown mode'):
        validate_request('unknown', 'false')


def test_invalid_boolean():
    with pytest.raises(RuntimeError, match='enable_sim must be true or false'):
        validate_request('sim_only', 'maybe')


def test_sim_only_enabled():
    validate_request('sim_only', 'true')


@pytest.mark.parametrize("mode", ["mapping", "localization", "navigation"])
def test_w09_w11_modes_enabled(mode):
    validate_request(mode, "true")

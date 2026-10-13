from unittest.mock import patch

import pytest

from t1_mujoco.runtime import RuntimeSetupError, import_dependency, load_task_config, require_path, validate_selection


@pytest.mark.parametrize('mode,task,match', [
    ('unknown', 't1_walk', 'mode must be'),
    ('sim_only', 'other', 'task must be t1_walk'),
])
def test_invalid_selection(mode, task, match):
    with pytest.raises(RuntimeSetupError, match=match):
        validate_selection(mode, task)


@pytest.mark.parametrize('value,match', [('', 'is required'), ('relative/path', 'must be absolute')])
def test_bad_path_parameter(value, match):
    with pytest.raises(RuntimeSetupError, match=match):
        require_path(value, 'upstream_path')


def test_missing_path(tmp_path):
    with pytest.raises(RuntimeSetupError, match='upstream_path does not exist'):
        require_path(str(tmp_path / 'missing'), 'upstream_path')


def test_wrong_path_type(tmp_path):
    file = tmp_path / 'file'
    file.write_text('not a directory')
    with pytest.raises(RuntimeSetupError, match='wrong type'):
        require_path(str(file), 'upstream_path')


def test_incomplete_repository_fails_before_import(tmp_path):
    with patch('t1_mujoco.runtime.importlib.import_module') as importer:
        with pytest.raises(RuntimeSetupError, match='upstream_path/booster_deploy/utils/registry.py'):
            load_task_config(str(tmp_path))
        importer.assert_not_called()


@pytest.mark.parametrize('error', [ModuleNotFoundError('no library'), OSError('ABI loader failure')])
def test_dependency_error_is_actionable(error):
    with patch('t1_mujoco.runtime.importlib.import_module', side_effect=error):
        with pytest.raises(RuntimeSetupError, match='Cannot import mujoco.*Source ROS'):
            import_dependency('mujoco')


def test_absolute_path_independent_of_cwd(tmp_path, monkeypatch):
    directory = tmp_path / 'upstream'
    directory.mkdir()
    monkeypatch.chdir('/')
    assert require_path(str(directory), 'upstream_path') == directory

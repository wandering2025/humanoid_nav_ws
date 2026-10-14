import hashlib
import yaml
import pytest
from t1_mujoco.recording import w09_provenance
from t1_mujoco.simulation import parameter_defaults, validate_parameters


def fixture(tmp_path):
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\n')
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\x00')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    meta = dict(map_yaml_sha256=sha(path), map_image_sha256=sha(image), scene_yaml_sha256='scene',
                T_world_map={'x': 0, 'y': 0, 'yaw': 0}, generated_code_sha='a'*40)
    path.with_suffix('.meta.yaml').write_text(yaml.safe_dump(meta))
    return path, {'layout': {'sha256': 'scene'}}


def test_saved_map_is_bound_to_scene_and_manifest(tmp_path):
    path, scene = fixture(tmp_path)
    result = w09_provenance({'map_file': str(path)}, scene)
    assert result['map_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert result['map_generated_code_sha'] == 'a'*40


@pytest.mark.parametrize('change', ['yaml', 'image', 'scene'])
def test_map_or_scene_change_requires_regeneration(tmp_path, change):
    path, scene = fixture(tmp_path)
    if change == 'yaml':
        path.write_text('image: map.pgm\nresolution: 0.1\n')
    elif change == 'image':
        (tmp_path / 'map.pgm').write_bytes(b'changed')
    else:
        scene['layout']['sha256'] = 'different'
    with pytest.raises(ValueError, match='checksum|scene'):
        w09_provenance({'map_file': str(path)}, scene)


def test_localization_cannot_start_without_saved_map():
    params = parameter_defaults()
    params['mode'] = 'localization'
    with pytest.raises(ValueError, match='saved map_file'):
        validate_parameters(params)


@pytest.mark.parametrize('mode', ['mapping', 'localization'])
def test_w09_cannot_use_policy_debug_bypass(mode):
    params = parameter_defaults()
    params.update(mode=mode, enable_policy_cmd_input=True)
    with pytest.raises(ValueError, match='forbid policy debug'):
        validate_parameters(params)

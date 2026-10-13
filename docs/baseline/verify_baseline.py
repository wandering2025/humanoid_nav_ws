"""Read-only W01 checks. Run from workspace after sourcing ROS then venv."""
import hashlib
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

import booster_assets
import mujoco
import numpy
import onnxruntime
import rclpy
import torch
import yaml

workspace = Path('/home/haoyang/humanoid_nav_ws')
print('start_utc', datetime.now(timezone.utc).isoformat())
print('tested_head', subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip())
assert Path.cwd() == workspace
assert os.getenv('WSL_DISTRO_NAME') == 'Ubuntu-24.04'
assert 'VERSION_ID="24.04"' in Path('/etc/os-release').read_text()
assert sys.prefix == '/home/haoyang/venvs/humanoid_mujoco'
assert os.getenv('ROS_DISTRO') == 'jazzy'
assert Path(rclpy.__file__).is_relative_to('/opt/ros/jazzy')
assert sys.version_info[:3] == (3, 12, 3)
for name, version in {'mujoco': '3.15.0', 'torch': '2.14.1', 'numpy': '2.5.4',
                      'onnxruntime': '1.31.0', 'booster-assets': '1.0.0'}.items():
    assert metadata.version(name) == version, name
assert torch.__version__ == '2.14.1+cu130'
manifest = json.loads((workspace / 'docs/evidence/W01/c2-assets.json').read_text())
dependencies = yaml.safe_load((workspace / 'dependencies.repos').read_text())['repositories']
for name, repo in manifest['repositories'].items():
    def git(*args):
        return subprocess.check_output(['git', '-C', repo['path'], *args], text=True).strip()
    assert git('rev-parse', 'HEAD') == repo['sha']
    assert git('status', '--porcelain') == repo['dirty'] == ''
    assert git('remote', 'get-url', 'origin') == repo['remote']
    assert dependencies[name] == {'type': 'git', 'url': repo['remote'], 'version': repo['sha']}
    assert hashlib.sha256(Path(repo['license_path']).read_bytes()).hexdigest() == repo['license_sha256']
for asset in manifest['files'].values():
    path = Path(asset['path'])
    assert path.stat().st_size == asset['size_bytes']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == asset['sha256']
assert Path(booster_assets.BOOSTER_ASSETS_DIR) == Path(manifest['repositories']['booster_assets']['path'])
assert (Path(booster_assets.BOOSTER_ASSETS_DIR) / 'robots/T1/meshes').is_dir()
model = mujoco.MjModel.from_xml_path(manifest['files']['mjcf']['path'])
for field in ('nq', 'nv', 'nu', 'njnt', 'nbody', 'ngeom'):
    assert getattr(model, field) == manifest['model'][field]
for field, kind, count in [('joint_names', mujoco.mjtObj.mjOBJ_JOINT, model.njnt),
                           ('actuator_names', mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)]:
    assert [mujoco.mj_id2name(model, kind, i) for i in range(count)] == manifest['model'][field]
print('python', sys.executable, 'torch_runtime', torch.__version__, 'cuda_available', torch.cuda.is_available())
print('PASS environment/imports/repos/paths/hashes/MJCF/dependencies.repos')
print('end_utc', datetime.now(timezone.utc).isoformat())

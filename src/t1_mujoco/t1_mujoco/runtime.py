"""Process-local dependency loading. Importing this module starts no physics."""

from copy import deepcopy
import importlib
from pathlib import Path
import sys

MODES = ('sim_only', 'mapping', 'localization', 'navigation', 'evaluation')


class RuntimeSetupError(RuntimeError):
    """A runtime dependency or user configuration could not be used."""


def validate_selection(mode, task):
    if mode not in MODES:
        raise RuntimeSetupError(f'mode must be one of {MODES}; got {mode!r}')
    if task != 't1_walk':
        raise RuntimeSetupError(f'task must be t1_walk; got {task!r}')


def require_path(value, label, *, directory=True):
    if not value:
        raise RuntimeSetupError(f'{label} is required; supply an absolute path.')
    path = Path(value).expanduser()
    if not path.is_absolute():
        raise RuntimeSetupError(f'{label} must be absolute; got {value!r}')
    if not (path.is_dir() if directory else path.is_file()):
        raise RuntimeSetupError(f'{label} does not exist or has the wrong type: {path}')
    return path.resolve()


def import_dependency(name):
    try:
        return importlib.import_module(name)
    except Exception as exc:
        raise RuntimeSetupError(
            f'Cannot import {name} with {sys.executable}: {exc}. '
            'Source ROS and the workspace environment; verify T1_VENV against the W01 baseline.'
        ) from exc


def load_task_config(upstream_path, task='t1_walk'):
    """Import only the T1 registration module, then copy the shared config.

    The path is inserted only into this Python process, never shell PYTHONPATH.
    No controller, model, policy or viewer is instantiated here.
    """
    validate_selection('sim_only', task)
    root = require_path(upstream_path, 'upstream_path')
    for relative in ('booster_deploy/utils/registry.py', 'tasks/locomotion/robots/t1/__init__.py'):
        require_path(str(root / relative), f'upstream_path/{relative}', directory=False)
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    module = import_dependency('tasks.locomotion.robots.t1')
    registry = import_dependency('booster_deploy.utils.registry')
    for imported in (module, registry):
        if not Path(imported.__file__).resolve().is_relative_to(root):
            raise RuntimeSetupError(f'Upstream import shadowed by {imported.__file__}; expected {root}')
    try:
        return deepcopy(registry.get_task(task))
    except Exception as exc:
        raise RuntimeSetupError(f'Cannot load task {task} from {root}: {exc}') from exc


def inspect_runtime(upstream_path, assets_path, mode, task):
    validate_selection(mode, task)
    upstream = require_path(upstream_path, 'upstream_path')
    modules = {name: import_dependency(name) for name in ('rclpy', 'numpy', 'mujoco', 'torch', 'booster_assets')}
    installed_assets = require_path(str(modules['booster_assets'].BOOSTER_ASSETS_DIR), 'booster_assets directory')
    assets = require_path(assets_path, 'assets_path') if assets_path else installed_assets
    if assets != installed_assets:
        raise RuntimeSetupError(
            f'assets_path {assets} differs from installed booster_assets {installed_assets}; '
            'the frozen upstream resolves assets through that package.'
        )
    cfg = load_task_config(str(upstream), task)
    mjcf = require_path(cfg.robot.mjcf_path.format(BOOSTER_ASSETS_DIR=str(assets)), 'T1 MJCF', directory=False)
    # Upstream Policy.task_path is the directory containing its constructor.
    policy_module = import_dependency(cfg.policy.constructor.__module__)
    checkpoint = Path(cfg.policy.checkpoint_path)
    if not checkpoint.is_absolute():
        checkpoint = Path(policy_module.__file__).parent / checkpoint
    checkpoint = require_path(str(checkpoint), 'T1 policy checkpoint', directory=False)
    return {
        'upstream_path': str(upstream), 'assets_path': str(assets),
        'mjcf_path': str(mjcf), 'checkpoint_path': str(checkpoint),
        'libraries': {name: {'version': getattr(module, '__version__', 'n/a'), 'path': module.__file__}
                      for name, module in modules.items()},
        'joint_count': len(cfg.robot.joint_names),
        'policy_dt': cfg.policy_dt, 'decimation': cfg.mujoco.decimation,
        'physics_dt': cfg.mujoco.physics_dt,
        'physics_started': False,
    }

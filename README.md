# Booster T1 MuJoCo navigation

This workspace implements the project one explicitly assigned step at a time. W03 provides four buildable ROS 2 Jazzy packages and a one-shot runtime diagnostic. The physics runner starts in W04; robot description, LiDAR, SLAM and Nav2 remain unimplemented and disabled.

The frozen external repositories and existing venv are documented in [environment baseline](docs/baseline/environment.md) and [dependencies.repos](dependencies.repos). Design contracts: [system](docs/architecture/system.md), [interfaces](docs/architecture/interfaces.md), [acceptance](docs/architecture/acceptance.md). Progress and handoff: [execution board](docs/plans/t1_mujoco_navigation/执行看板.md), [W03 record](docs/plans/t1_mujoco_navigation/records/W03.md).

| Package | Build type | W03 content |
| --- | --- | --- |
| t1_mujoco | ament_python | Runtime loading, ROS diagnostic, installed parameter file |
| t1_description | ament_cmake | Resource location; description reserved for W06/W07 |
| t1_navigation | ament_cmake | Resource location; navigation parameters reserved for W09–W11 |
| t1_bringup | ament_python | Launch composition; Python evaluation tools reserved for W13 |

## Enter the environment and build

Set external paths once in each new Bash terminal. These are this machine's bindings; substitute your own paths when moving to another machine. Do not install the full W01 snapshot as a pip requirements file.

```bash
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
# Optional: T1_ASSETS; otherwise use the already installed booster_assets package.
source /home/haoyang/humanoid_nav_ws/scripts/env.sh
cd "$T1_WS"
colcon list --base-paths src
colcon build --base-paths src --event-handlers console_direct+
source "$T1_WS/scripts/env.sh"  # reload the newly installed overlay
```

The source script loads ROS → existing workspace overlay → venv without changing cwd, `.bashrc` or dependencies. It preserves ROS `PYTHONPATH`. `T1_ROS_SETUP` optionally overrides `/opt/ros/jazzy/setup.bash`.

System `/usr/bin/colcon` builds with system Python. Its generated `t1_diagnostic_python` console script has a system shebang and is an internal packaging entrypoint. The public executable `t1_diagnostic` explicitly runs `"$T1_VENV/bin/python" -m t1_mujoco.diagnostic`; `exec` preserves signals and exit status. The venv has MuJoCo/PyTorch and receives apt rclpy through the sourced ROS environment. This avoids installing colcon/pytest into the external venv or copying system site-packages. Future simulation entrypoints must use the same interpreter rule.

## Run the installed diagnostic

```bash
ros2 run t1_mujoco t1_diagnostic
ros2 launch t1_bringup diagnostic.launch.py
# The W03 top-level skeleton also runs only this one-shot diagnostic:
ros2 launch t1_bringup bringup.launch.py mode:=sim_only use_sim_time:=true
```

All commands work outside the workspace after sourcing the environment. The diagnostic prints one `T1_DIAGNOSTIC` JSON line containing the actual `sys.executable`, Python/ROS/library versions, external paths and copied T1 task timing. It imports rclpy, NumPy, MuJoCo, PyTorch and the T1 registration module; it never constructs a model, policy or controller, opens a viewer or advances physics. It then destroys the ROS node and exits. It does not publish `/clock`, TF or fake success for simulation. A child failure propagates to a nonzero launch exit.

Arguments: `mode` (`sim_only|mapping|navigation|evaluation`), `use_sim_time`, `upstream_path`, optional `assets_path`, `task` (only `t1_walk`), `params_file`. These select diagnostic context in W03. `enable_sim:=true` fails clearly until W04 implements `t1_sim`; selecting a future mode starts no SLAM/Nav2. The parameter file is installed at `share/t1_mujoco/config/runtime.yaml`. Runtime path insertion is confined to the diagnostic Python process. `booster_deploy` has no pip packaging metadata and is not installed editable. Configuration is deep-copied from its registry before use. Explicit asset paths must match the installed frozen `booster_assets` package.

## Verify W03

```bash
cd "$T1_WS"
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --all --verbose
python docs/evidence/W03/verify_w03.py
```

The last command exercises the installed `ros2 run` and `ros2 launch` paths from `/tmp`, checks interpreter/library evidence, tests error exits, verifies installed resources and frozen upstream cleanliness. It saves small logs and a timestamped summary under [W03 evidence](docs/evidence/W03/). CMake resource packages intentionally have no tests; the Python packages contain real parameter/path/dependency and unimplemented-entrypoint tests. Build/install/log outputs and large run data are ignored. No physics or GUI acceptance is claimed in W03.

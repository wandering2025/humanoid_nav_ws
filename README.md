# Booster T1 MuJoCo navigation

This workspace implements the project one explicitly assigned step at a time. W01–W09 provide the physical strategy runner, calibrated command guard, consistent ROS state/TF, static scenes, compensated LiDAR, SLAM mapping and saved-map AMCL localization. The W10 safety chain and W11 autonomous Nav2 navigation are not yet implemented.

The frozen external repositories and existing venv are documented in [environment baseline](docs/baseline/environment.md) and [dependencies.repos](dependencies.repos). Design contracts: [system](docs/architecture/system.md), [interfaces](docs/architecture/interfaces.md), [acceptance](docs/architecture/acceptance.md). Progress and handoff: [execution board](docs/plans/t1_mujoco_navigation/执行看板.md), [W09 record](docs/plans/t1_mujoco_navigation/records/W09.md).

| Package | Build type | Current content |
| --- | --- | --- |
| t1_mujoco | ament_python | Physical strategy runner, calibrated command guard, state/TF, scenes and compensated LiDAR |
| t1_description | ament_cmake | Frozen external description wrapper, validated FK/mesh, footprint and RViz |
| t1_navigation | ament_cmake | SLAM/AMCL configuration, LiDAR-built map and provenance; W10/W11 follow |
| t1_bringup | ament_python | Simulation/mapping/localization launch, one-shot AMCL initialization; W13 evaluation follows |

## Enter the environment and build

Set external paths once in each new Bash terminal. These are this machine's bindings; substitute your own paths when moving to another machine. Do not install the full W01 snapshot as a pip requirements file.

```bash
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
# Optional: T1_ASSETS; otherwise use the already installed booster_assets package.
source /home/haoyang/humanoid_nav_ws/scripts/env.sh
cd "$T1_WS"
colcon list --base-paths src
/usr/bin/colcon build --base-paths src --event-handlers console_direct+
source "$T1_WS/scripts/env.sh"  # reload the newly installed overlay
```

The source script loads ROS → existing workspace overlay → venv without changing cwd, `.bashrc` or dependencies. It preserves ROS `PYTHONPATH`. `T1_ROS_SETUP` optionally overrides `/opt/ros/jazzy/setup.bash`.

System `/usr/bin/colcon` builds with system Python. Its generated `t1_diagnostic_python` console script has a system shebang and is an internal packaging entrypoint. The public executable `t1_diagnostic` explicitly runs `"$T1_VENV/bin/python" -m t1_mujoco.diagnostic`; `exec` preserves signals and exit status. The venv has MuJoCo/PyTorch and receives apt rclpy through the sourced ROS environment. This avoids installing colcon/pytest into the external venv or copying system site-packages. The public `t1_sim` wrapper uses the same rule.

## Run the installed diagnostic

```bash
ros2 run t1_mujoco t1_diagnostic
ros2 launch t1_bringup diagnostic.launch.py
# The W03 top-level skeleton also runs only this one-shot diagnostic:
ros2 launch t1_bringup bringup.launch.py mode:=sim_only use_sim_time:=true
```

All commands work outside the workspace after sourcing the environment. The diagnostic prints one `T1_DIAGNOSTIC` JSON line containing the actual `sys.executable`, Python/ROS/library versions, external paths and copied T1 task timing. It imports rclpy, NumPy, MuJoCo, PyTorch and the T1 registration module; it never constructs a model, policy or controller, opens a viewer or advances physics. It then destroys the ROS node and exits. It does not publish `/clock`, TF or fake success for simulation. A child failure propagates to a nonzero launch exit.

Arguments: `mode` (`sim_only|mapping|localization|navigation|evaluation`), `use_sim_time`, `upstream_path`, optional `assets_path`, `task` (only `t1_walk`), `params_file`. These select diagnostic context in W03. `enable_sim:=true mode:=sim_only` starts the W04 runner. Mapping and localization have dedicated W09 entries below; navigation/evaluation fail explicitly until W10/W11. The parameter file is installed at `share/t1_mujoco/config/runtime.yaml`. Runtime path insertion is confined to the diagnostic Python process. `booster_deploy` has no pip packaging metadata and is not installed editable. Configuration is deep-copied from its registry before use. Explicit asset paths must match the installed frozen `booster_assets` package.

## Verify packages (W03 baseline)

```bash
cd "$T1_WS"
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --all --verbose
# Historical W03 verification assumes simulation is unimplemented; use W04 verification below now.
```

The last command exercises the installed `ros2 run` and `ros2 launch` paths from `/tmp`, checks interpreter/library evidence, tests error exits, verifies installed resources and frozen upstream cleanliness. It saves small logs and a timestamped summary under [W03 evidence](docs/evidence/W03/). CMake resource packages intentionally have no tests; the Python packages contain real parameter/path/dependency and unimplemented-entrypoint tests. Build/install/log outputs and large run data are ignored. No physics or GUI acceptance is claimed in W03.


## Run the W04 simulation

This WSL host needs the explicitly enabled local TCP discovery path for cross-process ROS (see [ADR-009 investigation](docs/evidence/W04/dds-investigation.md)). In **each** terminal, after setting the external paths above:

```bash
export T1_LOCAL_DDS=1
source /home/haoyang/humanoid_nav_ws/scripts/env.sh
```

In one dedicated terminal, keep `"$T1_WS/scripts/start_local_dds.sh"` running in the foreground. All ROS terminals then use `TCPv4:[127.0.0.1]:42114`. Stop this server with Ctrl+C after the simulation and clients exit. For separate concurrent systems, use a distinct `T1_DDS_PORT` consistently; discovery-server isolation is by server, not only ROS_DOMAIN_ID. This setting is opt-in and does not edit `.bashrc` or system networking.

After building, in another sourced terminal:

```bash
ros2 launch t1_bringup simulation.launch.py
# Bounded headless standing test; target_rtf=0 means unlimited physics speed:
ros2 launch t1_bringup simulation.launch.py target_rtf:=0.0 max_sim_time:=60.0
# Optional viewer; Space pauses/resumes, window close or Ctrl+C exits:
ros2 launch t1_bringup simulation.launch.py headless:=false
# Experimental policy-unit commands (not physical velocity):
ros2 launch t1_bringup simulation.launch.py enable_policy_cmd_input:=true
# In a second sourced terminal, after /t1/sim_state reports READY:
ros2 topic pub -r 20 /t1/debug/policy_cmd geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
# Stop the publisher: either 0.5 s wall time or sim time timeout makes u_pol=0.
ros2 service call /t1/sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /t1/sim/pause std_srvs/srv/SetBool '{data: false}'
ros2 service call /t1/sim/reset std_srvs/srv/Trigger '{}'
```

The robot walks through joint torques and contact dynamics. Zero command continues the official standing policy. W04 initially rejected physical motion on `/cmd_vel` (`UNCALIBRATED`). Since W05, the launch loads the calibrated physical velocity map by default; an empty `command_map_file` still rejects physical commands. The debug input is opt-in and mutually exclusive with `/cmd_vel`; its values are policy units, bounded by the upstream ±1 limits. There is no minimum-speed boost.

Default startup warmup is 2 s of simulation time, frozen by W05. Commands received before READY or during pause are discarded. Pause retains physical state and policy history; resume clears commands. Reset is limited to sim_only, restores the initial pose, clears policy history/actions/counters, increments episode, and advances the clock origin by 0.02 s instead of jumping backwards. Outside sim_only restart the entire future stack. Fault/fall freezes physics and exits nonzero; no automatic recovery is claimed.

Each run creates a unique ignored `run_data/<run_id>/` containing `manifest.json`, `sim_trace.csv`, and `events.jsonl`. `max_sim_time` is an absolute simulation clock limit (0 = unlimited); `target_rtf` paces deadlines without skipping physics. Parameters are startup-only. No odometry, TF, LiDAR, SLAM, navigation or calibrated stopping distances are implemented in W04.

W04 reproduction scripts and results are in [docs/evidence/W04](docs/evidence/W04/README.md). Use an isolated ROS domain for tests; `ROS_LOG_DIR=$T1_WS/run_data/w04_ros_logs` keeps logs in the workspace. Only the test's own processes are stopped.

## W06 state visualization

With the environment and local discovery server configured above:

```bash
ros2 launch t1_bringup simulation.launch.py rviz:=true
# After READY, in another sourced terminal (physical units):
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}}'
```

Simulation launch now starts robot_state_publisher with use_sim_time=true and
wraps the frozen external T1 URDF in memory. RViz uses odom as Fixed Frame.
The model and base_link axes follow actual simulated joints and trunk pose.
`/odom` is ideal truth projected onto z=0, with child-frame twist;
`/t1/eval/ground_truth` is full trunk truth for evaluation only.
Covariance values are nominal, not measured estimation uncertainty.
sim_only has no map→odom; W09 mapping/localization add their exclusive estimator.
`/imu/data` is optional and currently absent.

Clock, odom, the two base TF edges, joint states and truth share the same
post-step snapshot stamp. They publish at 50 Hz simulation time by default.
`joint_state_divisor` in runtime.yaml allows positive integer policy-step
subdivision. Pause freezes dynamic publishing; resume clears commands.
A sim_only reset preserves monotonic clock and notifies a new episode;
consumers must avoid integrating/interpolating across that pose jump.
Restart the whole launch and consumers for a new run.

The calibrated physical command map loads by default (W05); 0<vx<0.315 m/s
is mapped to zero, so it does not support arbitrarily slow creeping.
W06 reproduction and measured topic/TF/time evidence: [W06 evidence](docs/evidence/W06/README.md).


## W09 mapping and saved-map localization

With the environment and local DDS server above configured in each terminal:

```bash
ros2 launch t1_bringup mapping.launch.py rviz:=true
# Separate terminal, after READY: controlled development teleop in physical units.
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.4 -p turn:=0.6 -p stamped:=false
# Keep clear of walls; W10 full safety chain is not implemented yet.
# Save the live SLAM map to a new path (do not overwrite the versioned map casually):
ros2 run nav2_map_server map_saver_cli -f "$T1_WS/run_data/my_map" --ros-args -p use_sim_time:=true
```

Stop the mapping launch and teleop before starting localization. The shipped map
is tied to static_world and includes its metadata; a custom map needs matching
metadata before it can be used by this project's localization launch.

```bash
ros2 launch t1_bringup localization.launch.py rviz:=true
# After lifecycle activation, initialize once with an approximate pose in map:
ros2 run t1_bringup initial_pose 0.2 -0.15 0.15
# Or use RViz's 2D Pose Estimate. Yaw is radians; do not repeatedly reset from truth.
```

Localization starts map_server then AMCL; mapping only starts SLAM Toolbox.
No mode includes an autonomous planner/controller. Map/scene checksum mismatch
fails startup. Restart the complete launch to change modes or reset simulation.
The localization demonstration uses ideal truth odometry and truth attitude
compensation; it does not establish robustness against real odometry drift.
Reproduction, route, convergence and map checks: [W09 evidence](docs/evidence/W09/README.md).

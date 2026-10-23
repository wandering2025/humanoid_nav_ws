# Setup and commands

## Environment

The recorded simulation environment uses Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, and MuJoCo 3.15.0. The headless launcher targets Linux x86_64 with **Landlock ABI 5 or newer** for per-process GPU device isolation. The graphical launcher includes a **WSL2 / WSLg / NVIDIA RTX 4050** rendering and host-monitoring profile. Use headless execution on other display configurations; adapting the graphical profile is a separate integration task.

The commands below provision a new environment. For an existing installation, set the three `T1_*` paths and continue at **Build**.

## Install dependencies

Install ROS 2 Jazzy with the desktop and development packages using the ROS distribution's Ubuntu installation instructions. Then, from a Bash terminal at the root of this repository:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake python3-venv python3-colcon-common-extensions \
  python3-vcstool python3-rosdep python3-pytest qtbase5-dev \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 ros-jazzy-robot-state-publisher \
  ros-jazzy-fastrtps imagemagick x11-utils mesa-utils

# Keep external repositories outside this workspace's src/ directory.
mkdir -p "$HOME/robot_sim_sources"
vcs import "$HOME/robot_sim_sources" < dependencies.repos

export T1_UPSTREAM="$HOME/robot_sim_sources/booster_deploy"
export T1_ASSETS="$HOME/robot_sim_sources/booster_assets"
export T1_VENV="$HOME/venvs/humanoid_mujoco"
python3 -m venv "$T1_VENV"
"$T1_VENV/bin/python" -m pip install -r requirements-sim.txt
"$T1_VENV/bin/python" -m pip install -e "$T1_ASSETS"
```

`dependencies.repos` pins the Booster deployment repository and robot assets to the recorded revisions. The walking checkpoint is `tasks/locomotion/robots/t1/models/t1_walk.pt` inside `booster_deploy`. The project loads that repository directly; it does not require installing `booster_deploy` as a Python package.

Keep the external repositories at their pinned revisions. Startup verifies their Git identity and checks the calibrated runtime files against `scripts/advanced/runtime-lock.json`.

## Build

```bash
# Set these in each new terminal, or use your own absolute paths.
export T1_VENV="$HOME/venvs/humanoid_mujoco"
export T1_UPSTREAM="$HOME/robot_sim_sources/booster_deploy"
export T1_ASSETS="$HOME/robot_sim_sources/booster_assets"

source scripts/env.sh
# Run sudo rosdep init once if rosdep has not been initialized on this machine.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
/usr/bin/colcon build --symlink-install --base-paths src \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source scripts/env.sh
ros2 run t1_mujoco t1_diagnostic
```

The environment loader sources ROS, the workspace overlay, and the Python environment in that order. System colcon builds the packages; simulation executables explicitly use `$T1_VENV/bin/python`. Run these commands from the repository root. `T1_ROS_SETUP` overrides `/opt/ros/jazzy/setup.bash` when needed.

## Run exploration

```bash
./scripts/run_exploration_demo.sh --preflight
./scripts/run_exploration_demo.sh --headless
```

Preflight builds the local ROS helper executables and checks configuration without starting a mission. A headless mission starts physical simulation, continuous SLAM, exploration, Nav2, and the moving obstacle. It writes a unique result directory under `run_data/exploration/`.

On the documented graphics profile:

```bash
./scripts/run_exploration_demo.sh          # MuJoCo + RViz
./scripts/run_exploration_demo.sh --hold   # Retain the final view
```

| Option | Behavior |
| --- | --- |
| `--static` | Remove the dynamic obstacle for a control run |
| `--no-rviz` | Show MuJoCo only; uses the same graphical profile |
| `--task /absolute/path/task.yaml` | Load a relative destination from another task file |
| `--static --scene exploration_south` | Explore the alternate static layout |
| `--time-limit 300` | Set the mission budget in simulation seconds |

The task schema is `schema: 1`, `frame: start`, and `goal: {x, y}`. Distances are metres in the starting body frame. The default is +11.0 m forward and +0.5 m left. Edit the file between runs; the launcher saves an immutable task snapshot at startup.

A normal run holds the final graphical view for eight seconds, requests a physical stop, and cleans up. Ctrl+C uses the same stop-and-cleanup path. Wait for the launcher to return before starting another mission; TCP port recovery can take up to 65 seconds. The launcher owns DDS port 42127 and ROS domain 37.

Read `run_data/exploration/<run-id>/final-result.json` for the overall result, arrival error, contact counts, and each component's exit status. Runtime logs, map snapshots, and raw traces stay outside Git.

## Guided demonstration

```bash
./scripts/run_advanced_demo.sh --check-goal
./scripts/run_advanced_demo.sh --headless
```

This mode uses configured mapping viewpoints and reads `presentation/goal.yaml`, an **absolute map-frame** destination. Its accepted rectangle is x∈[11,12] m and y∈[−0.5,0.5] m, with a fixed south-facing final heading. Exploration uses its own task file and chooses observation targets online.

## Development checks

```bash
./scripts/test.sh
python3 scripts/check_public.py
```

These run the source tests and public-documentation checks. Physical mission evaluation uses the launch commands above. See [validation](validation.md) for the recorded mission and load-test results.

# Exploration demo commands

The Bash sections below require an Ubuntu / WSL Bash terminal. A `PS ...` prompt is Windows PowerShell, even when the current folder is under `\\wsl$`. Changing into a WSL shared folder does not select Bash. The launcher loads the existing ROS, overlay and Python environment automatically.

## Start from Windows PowerShell

```powershell
# Start MuJoCo + RViz, online exploration and the real dynamic obstacle.
wsl.exe -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws_advanced --exec bash ./scripts/run_exploration_demo.sh
```

This command works from any PowerShell directory. Append `--hold` to keep the stopped scene visible after arrival; otherwise the windows remain for eight seconds. Startup prints progress. Press Ctrl+C once in the launching terminal and wait for cleanup.

Alternatively, enter Ubuntu interactively, then use the Bash commands below:

```powershell
wsl.exe -d Ubuntu-24.04
```

## Before the presentation

```bash
cd /home/haoyang/humanoid_nav_ws_advanced

# Optional: edit B relative to A for the next run (default x=11.0, y=0.5).
nano src/t1_navigation/config/exploration_task.yaml

# Show supported options without starting simulation.
./scripts/run_exploration_demo.sh --help
```

See [coordinate rules](GOAL_CONFIGURATION.md). Save the task file before starting; coordinate changes need no rebuild. This entry does not read `presentation/goal.yaml` and does not offer `--check-goal`: reachability must be discovered from online observations. Optional `./scripts/run_exploration_demo.sh --preflight` checks the task/environment without starting a mission; it creates a run directory and is not a reachability or GUI test.

## Start

```bash
# Recommended: MuJoCo + RViz + autonomous exploration + EXP01 obstacle.
./scripts/run_exploration_demo.sh

# Keep the final stopped scene visible until Ctrl+C.
./scripts/run_exploration_demo.sh --hold

# Optional complete run without graphical windows.
./scripts/run_exploration_demo.sh --headless

# MuJoCo window only.
./scripts/run_exploration_demo.sh --no-rviz

# Explicit control run: no dynamic entity, no windows.
./scripts/run_exploration_demo.sh --headless --static
```

Start only one mission at a time, including the legacy demo: both use the same mission lock and DDS port 42127 / ROS domain 37. The terminal prints the W16 run ID and output directory. MuJoCo shows physical walking; RViz shows online SLAM, raw LiDAR, the latest Nav2 plan (blue), the selected target (orange), travelled motion (purple) and mission state. The plan can be retained while waiting. A/B markers appear after the explorer locks the actual starting frame.

Use `--task /absolute/path/task.yaml` for another task. `--time-limit SECONDS` sets the total simulation-time budget (default 600; valid range greater than 0 and at most 800). `--scene NAME_OR_YAML` is simulator-only; EXP01 requires the default `advanced_world`, so other layouts require `--static`. A changed task/layout is not guaranteed to reproduce the validated obstacle encounter or arrival.

## Stop and finish

Press **Ctrl+C once in the launching terminal**. This requests latched safety stop and bounded cleanup. Wait for the process to finish; DDS TCP TIME_WAIT may add up to 65 seconds. Interrupting before arrival reports FAILED/CANCELED. Normal completion prints `Final SUCCESS` after verified arrival, zero contact/fall checks, cleanup and port release.

Do not use global `pkill`, close the terminal during cleanup, or move the robot with the keyboard. Stop the current run before editing B for another run. Starting the demo does not start recording. Use the terminal's final result to confirm arrival; an RViz status label may retain an earlier rendered state.

## Inspect this run's result

```bash
cd /home/haoyang/humanoid_nav_ws_advanced
# Set this to the output directory printed by the launch you want to inspect.
exploration_run=run_data/exploration/REPLACE_WITH_PRINTED_RUN_ID
cat "$exploration_run/task.yaml"
cat "$exploration_run/explorer/result.json"
cat "$exploration_run/final-result.json"
```

`task.yaml` is the immutable input snapshot. `explorer/result.json` records the latched A, resolved B and exploration outcome; `explorer/decisions.jsonl` records selected targets and actual paths. `final-result.json` includes the physical arrival, contact/fall and cleanup checks. A preflight-only or startup-rejected directory may not contain these result files. Raw runs stay local and ignored by Git.

## Legacy guided demo and backup

The old entry still follows preset observation waypoints and reads [goal.yaml](goal.yaml), with an absolute map goal and fixed final heading. It is separate from W16. Its SLAM corrections pause during dynamic navigation; W16 keeps SLAM running.

```bash
# Legacy configuration check, then optional old guided GUI demo.
./scripts/run_advanced_demo.sh --check-goal
./scripts/run_advanced_demo.sh --hold
```

[Open the published guided-demo backup video](../docs/evidence/advanced-optimization-visual/raw/recording-03/advanced-demo.mp4). It uses absolute map B=(12,0), independent of either current task file. It does not demonstrate W16 exploration. The local video is an ignored artifact; [release details and chapters](../docs/advanced/optimization-release-manifest.md) identify it.

```bash
# Original stable demo.
/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh
```

The stable command runs the separate stable workspace. It does not read the advanced presentation goal. For a new recording, follow the [independent capture procedure](../docs/advanced/recording.md).

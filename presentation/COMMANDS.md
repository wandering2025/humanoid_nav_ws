# Demo commands

All commands below run in an Ubuntu / WSL Bash terminal. The launcher loads the existing ROS, overlay and Python environment automatically.

## Before the presentation

```bash
cd /home/haoyang/humanoid_nav_ws_advanced

# Edit B for the next run.
nano presentation/goal.yaml

# Validate B and display the resolved goal, approach and configuration hash.
# No simulator, ROS graph or GPU renderer is started.
./scripts/run_advanced_demo.sh --check-goal
```

See [coordinate rules](GOAL_CONFIGURATION.md). Save the file before starting. No build is required after changing only goal.yaml.

## Start

```bash
# Recommended live demo: MuJoCo + RViz + autonomous mission.
./scripts/run_advanced_demo.sh

# Keep the final stopped scene visible until Ctrl+C.
./scripts/run_advanced_demo.sh --hold

# Optional complete run without graphical windows.
./scripts/run_advanced_demo.sh --headless
```

Start only one mission at a time. Watch the printed `Presentation B` and configuration SHA256 to confirm the selected target. MuJoCo shows physical walking; RViz shows LiDAR, map growth, actual plans, travelled motion and mission state. Blue is the last successful execution plan; its age is shown while waiting.

## Stop and finish

Press **Ctrl+C once in the launching terminal**. This requests latched safety stop and bounded cleanup. Wait for the process to finish; DDS TCP TIME_WAIT may add up to 65 seconds. An intentionally interrupted mission is reported FAILED/CANCELED, not as arrival success. Normal completion prints `Final SUCCESS` after verified arrival, zero contact/fall checks, cleanup and port release.

Do not use global `pkill`, close the terminal during cleanup, or move the robot with the keyboard. Stop the current run before editing B for another run. Starting the demo does not start recording.

## Inspect the latest result

```bash
cd /home/haoyang/humanoid_nav_ws_advanced
latest_run=$(ls -td run_data/advanced/*_advanced_* | head -n 1)
cat "$latest_run/presentation-goal.json"
cat "$latest_run/final-result.json"
```

The first file binds the exact configuration text, SHA256 and selected goal to the run. `final-result.json` exists after a complete cleanup; a validation-only or startup-rejected directory may not have one.

## Backup

[Open the published backup video](../docs/evidence/advanced-optimization-visual/raw/recording-03/advanced-demo.mp4). It uses B=(12,0), independent of the current goal.yaml.

```bash
# Original stable demo.
/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh
```

The stable command runs the separate stable workspace. It does not read the advanced presentation goal. For a new recording, follow the [independent capture procedure](../docs/advanced/recording.md).

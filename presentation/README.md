# Presentation kit

Start here: [Commands](COMMANDS.md) · [English / Chinese talk track](TALK_TRACK.md) · [B-point configuration](GOAL_CONFIGURATION.md).

The editable runtime interface is [goal.yaml](goal.yaml). Every advanced-demo launch reads it before simulation starts. The selected B, derived approach, both visual markers and arrival checks use the same per-run snapshot. Edit between runs; no rebuild is needed for coordinate changes.

Recommended start from an **Ubuntu / WSL Bash** terminal, from any directory:

```bash
/home/haoyang/humanoid_nav_ws_advanced/scripts/run_advanced_demo.sh
```

From **Windows PowerShell** (`PS ...` prompt), explicitly select WSL and Bash:

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws_advanced --exec bash ./scripts/run_advanced_demo.sh
```

Append `--hold` to keep the final scene visible. See [commands](COMMANDS.md) for validation and stopping.

Use the blue RViz line to explain the current Nav2 path, the cyan line for the full A-to-B overview and the purple line for travelled motion. The orange cart blocks the passage, the robot waits, then continues after it withdraws.

The published backup video and its chapters are linked in the [release manifest](../docs/advanced/optimization-release-manifest.md). It uses the original B=(12,0); editing goal.yaml changes the next live run, not that existing video.

Read [validation](VALIDATION.md) for the configurations actually tested. This is guided autonomous mapping with ideal odometry; SLAM corrections pause during dynamic navigation while live LiDAR obstacle avoidance continues.

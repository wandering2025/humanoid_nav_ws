# Presentation kit

Start here: [Commands](COMMANDS.md) · [English / Chinese talk track](TALK_TRACK.md) · [B-point configuration](GOAL_CONFIGURATION.md).

**Local reliability rehearsal (2026-10-20): FAILED.** The current version is not yet accepted for a reliable live demonstration under screen-sharing load. Local capture/encoding tests exposed intermittent scan-timeout stops and a DDS shutdown crash. [Current results and limits](VALIDATION.md#local-reliability-rehearsal-2026-10-20) supersede the earlier single-run success for readiness decisions. Actual Google Meet sharing was not run.

The default presentation is **W16: unknown-environment exploration with real dynamic obstacles**. Starting with a new SLAM session, the robot knows only B relative to A. It selects observation goals from its current map and uses Nav2 to reach B through observed free space. The user tested the graphical demo and approved this presentation update on 2026-10-20 (Europe/Berlin).

Recommended start from an **Ubuntu / WSL Bash** terminal, from any directory:

```bash
/home/haoyang/humanoid_nav_ws_advanced/scripts/run_exploration_demo.sh
```

From **Windows PowerShell** (`PS ...` prompt), explicitly select WSL and Bash:

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws_advanced --exec bash ./scripts/run_exploration_demo.sh
```

This opens **MuJoCo + RViz** and enables the EXP01 dynamic obstacle. After safe arrival, the windows stay open for eight seconds; append `--hold` to retain them until Ctrl+C. See [commands](COMMANDS.md) for options, stopping and results.

Edit [exploration_task.yaml](../src/t1_navigation/config/exploration_task.yaml) to set B in metres: +x is the robot's forward direction at A, +y is left. The default is (11.0, 0.5). Save between runs; no rebuild is needed. [goal.yaml](goal.yaml) remains the separate **legacy guided-demo** configuration and is not read by exploration.

In RViz, blue shows the latest Nav2 plan, orange the selected observation goal or B, and purple the travelled path. MuJoCo shows physical walking and the moving obstacle. The obstacle participates in collision detection and LiDAR; the validated run waited while blocked, then continued after it withdrew. SLAM remains active throughout.

Read [validation](VALIDATION.md) for measured results and limitations, including ideal odometry, limited LiDAR and a kinematic obstacle. [W16 evidence](../docs/evidence/W16/README.md) includes actual MuJoCo/RViz screenshots, maps and dynamic-response audits.

The [published backup video and chapters](../docs/advanced/optimization-release-manifest.md) show the **older guided route at absolute map B=(12,0)**. They are not a recording of W16 exploration. The [legacy command](COMMANDS.md#legacy-guided-demo-and-backup) remains available; no new exploration video was recorded.

# W11 autonomous static navigation evidence

All C1–C4 acceptance checks pass, with one recorded failed suite version (v1, below). This covers static obstacles only; no W12 dynamic obstacles were started.

The formal runs, cancel/replace test and startup probe all target the tested SHA `66f1069bef3d5120aa3acbcc8891cac78cb3ab4d`. During those runs only documentation/evidence and the user's `docs/start_commands.md` were dirty. Every src file still matches that commit (`final-verification.json`). Build, 108 unit tests (0 failures) and the audit ran against the same sources.

## Results (suite v2, `suite.json`)

Truth criteria, frozen in C2 at their initial values:
- position ≤ 0.30 m and yaw ≤ 20°,
- stationary for 2.0 s (W05 0.5 s window: < 0.05 m/s and < 0.05 rad/s),
- ≤ 120 s simulation time,
- Nav2 SUCCEEDED and verified by the goal tool,
- no environment contact and no fall.

| Case | Kind | Time (sim s) | Pos err (m) | Yaw err (°) | Terminal retries | Path (m) | Min centre / footprint clearance (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S01 | short straight | 6.3 | 0.107 | 1.8 | 0 | 1.93 | 2.29 / 1.71 |
| S02 | corner | 17.1 | 0.175 | 6.5 | 0 | 5.25 | 2.45 / 1.94 |
| S03 | island detour | 47.8 | 0.190 | 8.5 | 1 | 13.42 | 2.11 / 1.55 |
| S04 | full A(0,0)→B(9,0) | 75.8 | 0.033 | 3.4 | 1 | 18.89 | 2.15 / 1.57 |
| S05 | return detour + heading change | 54.7 | 0.153 | 3.5 | 2 | 14.27 | 2.16 / 1.68 |

S03–S05 straight lines are blocked by the island. Their first global plans deviate 2.97–2.99 m laterally from the straight line (9.3–11.1 m plans vs. 6.5–9 m straight). Across all runs:
- no `contact_events` with the environment or non-foot ground, and no FALLEN/FAULT;
- after arming, no Collision Monitor stop or slowdown and no gate latch;
- 0 TF extrapolation/old-data/timeout lines in navigation nodes, 0 empty scans;
- graph contracts pass: `/cmd_vel` is published only by `collision_monitor`, `/t1/cmd_navigation` only by the adapter, `/clock` only by `t1_sim`, and `map→odom` only by `amcl`. Truth is read only by the evaluator, and every topic is Twist only;
- parameter contracts pass: sim time, unstamped commands, `base_footprint`, Wait-only behaviors, and a 2.05 m costmap radius.

Benign log lines, recorded rather than hidden:
- S03 and S04 each logged one transient MPPI "Optimizer fail to compute path" warning, absorbed by `failure_tolerance` with no progress failure or abort.
- S04's RViz display filter dropped messages while its TF cache warmed up.
- In every run, the Collision Monitor reported "invalid source" for under 0.1 s before its first scan, which precedes arming.

## Visual evidence

- Plot: `static-runs.png` shows all five truth trajectories and the first plans on the saved SLAM map.
- Images: `w11_S04-midroute.png` and `w11_S04-final.png` are actual RViz images (inspected) showing the costmaps, plan, scan and W10 zones.
- Machine-readable data per case: `w11_Sxx.json` (all metrics, attempts, events, graph, parameters), `w11_Sxx-trajectory.csv` (truth, measured and commanded velocities, u_pol at 10 Hz), and `w11_Sxx-layers.csv`. The layers file holds the four layers at 10 Hz: Nav2 raw → adapter → gate → smoother → collision_monitor (final), plus v_guard/u_pol and measured velocity.

## Cancel / replacement (`w11_cancel_replace.json`, F09 with a real action)

The first goal was canceled at 0.49 m/s, and the adapter called `/t1/safety/stop`. The gate latched `EXPLICIT_STOP_OR_CANCEL` 0.02 s sim later, and the policy command reached zero 0.02 s after cancel. After zero, the robot moved at most 0.114 m (W10 envelope 0.453 m), and there was no motion until the explicit re-arm and a new goal.

A second goal was then preempted by a replacement. The old goal ended ABORTED (never SUCCEEDED), and the replacement SUCCEEDED, ending 0.09 m from its target.

## Version history and failures

- **Pre-check (ADR-015)**: the static_world v1 A→B was unreachable under W10's frozen zones. The user chose to adjust the scene. Feasibility: `scene_feasibility.py`. The v2 map was rebuilt by SLAM from real walking: `mapping.json` and `map-quality.json` (wall P95 0.052 m, completeness P95 0.053 m, 81/81 free samples). Three AMCL restarts were re-verified: `localization*.json`, converging in 1.3–1.9 s to 0.018–0.056 m.
- **C2**: 25 development runs, listed in `c2-dev-experiments.json`, each with its failure mode and fix (ADR-016). Development runs used dirty trees and are not acceptance.
- **Suite v1** (`suite-v1/`, tested at 3fcd2ac): S01, S02, S03 and S05 SUCCESS, S04 FAIL_STARTUP. The localization lifecycle manager hung on a lost map_server configure response, so v1 is FAIL and was kept.
- **v2 fix**: RViz now starts with the delayed group, and the lifecycle managers start 3 s after their nodes. The startup probe (`c3-startup-reliability.json`) then reached navigation-active in 8/8 launches with RViz on, with no lost responses. All 5 cases were re-run.

## Reproduce

Run these from a fresh terminal. Use no installs; each script owns DDS port 42121 and must not run concurrently.

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42121 ROS_DOMAIN_ID=31 ROS_LOG_DIR=$PWD/docs/evidence/W11/raw/ros-log
source scripts/env.sh && /usr/bin/colcon build --symlink-install --base-paths src && source scripts/env.sh
/usr/bin/colcon test --base-paths src && /usr/bin/colcon test-result --all --verbose
cd docs/evidence/W11
"$T1_VENV/bin/python" nav_run.py startup 8      # startup reliability
"$T1_VENV/bin/python" nav_run.py suite          # S01–S05, writes suite.json and per-case files
"$T1_VENV/bin/python" nav_run.py cancel         # cancel + replacement
MPLCONFIGDIR=/tmp/t1-w11-mpl /usr/bin/python3 plot_runs.py
cd ../../.. && python3 docs/evidence/W11/final_checks.py
```

The TF authority sidecar `docs/evidence/W09/raw/authority-build/tf_authority` is built as described in the W09 README. Rebuilding the map (`remap.py`, then `analyze_map.py` and `check_localization.py`) rewrites the tracked map files, so inspect the diff before committing.

Interactive use: `ros2 launch t1_bringup navigation.launch.py rviz:=true initial_pose:="0 0 0"`, then `ros2 run t1_bringup navigate_to_pose 9 0 0`. For RViz "2D Goal Pose", first call `ros2 service call /t1/safety/arm std_srvs/srv/Trigger` while zero is flowing.

## Limits

- **Simulation assumptions**: ideal truth odometry, truth attitude compensation, and a noiseless instantaneous 9-layer LiDAR (planar projection, not 3D occupancy). It is a software safety chain, not a certification.
- **Navigation footprint**: the 2.05 m circle plus W10's omnidirectional zones need passages ≥ 4.1 m wide, and goals need ≥ 2.05 m clearance plus a 1.5 m approach corridor.
- **In-place yaw alignment**: alone it is imprecise for this gait (7–28° measured). Arrival relies on the staged walking approach and up to 3 integral retries, and an unverified arrival counts as failure.
- **Runtime**: RTF median about 1.0 at target_rtf 1.0 (headless sim; RViz in S04 only).
- **Test scale**: five predefined cases with one run each; this is not W13's 10-run static suite.

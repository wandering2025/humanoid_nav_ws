# W08 LiDAR evidence and reproduction

Only W08: trunk-fixed 9×360 MuJoCo rays, true-attitude compensation, height filtering and 1° nearest-bin LaserScan. No SLAM, AMCL, Nav2 or W12 obstacle-motion implementation. Geometry, invalid-value conventions, grouping, TF, noise, coverage limits and truth assumptions: [sensor contract](../../architecture/lidar.md). Production code: `src/t1_mujoco/t1_mujoco/{lidar,lidar_config}.py`; integration in simulation/state_bridge/runner/recording; rigid mount in t1_bringup/description.py and LaserScan display in scene.rviz.

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
source scripts/env.sh
/usr/bin/colcon build --symlink-install
source scripts/env.sh
/usr/bin/colcon test --event-handlers console_direct+
/usr/bin/colcon test-result --verbose
$T1_VENV/bin/python docs/evidence/W08/check_rays.py
$T1_VENV/bin/python docs/evidence/W08/check_motion.py
MPLCONFIGDIR=/tmp/w08-matplotlib /usr/bin/python3 docs/evidence/W08/draw_walls.py
# Each following script owns and cleans up its dedicated server/processes.
export T1_LOCAL_DDS=1 T1_DDS_PORT=42118 ROS_DOMAIN_ID=78
export ROS_LOG_DIR=$PWD/run_data/w08_ros_logs
source scripts/env.sh
$T1_VENV/bin/python docs/evidence/W08/check_health.py
$T1_VENV/bin/python docs/evidence/W08/verify_ros.py
```

ROS/RViz requires local sockets and X11 on this host, following ADR-009. Keep port 42118 unused before each script. Simulation entrypoints explicitly use the existing venv; only standalone plotting uses installed system matplotlib. No dependency installation/upgrade or upstream modification. Raw logs, per-run manifests/traces, generated parameter copies and captured point arrays stay ignored in `raw/` and `run_data/`. Test scripts invoke the already-installed source packages and require the build above.

Manual visualization: with the existing dedicated DDS setup active, `ros2 launch t1_bringup simulation.launch.py scene:=calibration_world rviz:=true`. No movement is automatic. `/cmd_vel` remains physical desired velocity through W05's calibrated map. Configure optional point cloud/noise/dropout in a separate copy of runtime.yaml and pass its absolute `params_file` path; restart for changes. Geometry/ranges are frozen in LidarConfig and recorded in every manifest. User `docs/start_commands.md` was preserved.

Evidence map:

- `start-state.json`: branch, initial full SHA, user diff hash.
- `ray-tests.json`: 26 tests, installed mj_ray/mj_multiRay signatures, exact 1/3m faces, corner, 30° thin pole, tilted fixed fixture, ground/overhead, all robot descendants, manually moved independent mocap object, RNG, blind/unknown handling, owner guard, message serialization. Synthetic static fixture pose changes are calibration only, never robot walking.
- `motion.json`: paired full physical trajectories with/without LiDAR. Actual policy forward/turn/stop; no base-pose motion. Entire qpos/qvel sequence identical. Reports raw and quantized wall error, scan times, roll/pitch, coverage and physical displacement. Contact observations here are at policy steps; W07 already separately sampled every substep. `wall-overlay.png` plots actual ray-derived scan points in odom against independently known wall faces.
- `ros-rviz.json`: actual simulation/RSP/RViz, Reliable and BestEffort subscribers, scan/pointcloud/TF timestamp alignment, 10Hz simulation rate, dynamic virtual origin and rigid mount, motion, pause/resume/reset, W06 regressions, real executable path, orderly exits. `rviz-standing.png` and `rviz-moving.png` are actual window captures, visually inspected (Global Status OK, wall scans). Red arrows close to the robot are the existing Odometry display, not scan obstacles.
- `health.json`: actual ROS 100% dropout (unknown scans, zero policy command while continuing balance) and injected raycast exception at t=4s (no failed scan, FAULT sensor_exception, zero policy command, ERROR diagnostic, exit2). This is the W08 internal gate only, not W10 downstream sensor-timeout/collision-monitor testing.
- `*-attempt*.json`: retained failed development tests. Fixed-duration/edge-of-tolerance turn targets did not meet the physical 90±20° condition; feedback stop target was centered without changing thresholds. Remote FastDDS discovery reports history depth0: type/reliability/durability/owner checked remotely, factory-created local publisher confirms KeepLast5. Jazzy DiagnosticStatus.ERROR is bytes, so the probe uses the defined constant instead of integer2. Production sensor implementation did not change for these probe fixes.
- `source-audit.json`, `final-verification.json`, `process-audit.json`: tested SHA/source hashes, acceptance and cleanup summary; build/colcon text captures.

Limits: truth attitude and ideal truth odometry; instantaneous noiseless default; robot self-occlusion deliberately omitted; z=0 planar world and z∈[0.15,1.20]m projection; finite vertical/azimuth beams can miss small/low obstacles. Unknown/no-hit cannot be treated as proof of safe clearance. W09 must consume the stabilized frame and actual scan timestamps; W10 must account for blind regions, latency, invalid ranges and stopping distance. No later W step is started by these tests.

Final acceptance on source **9fa009bc50b601f41db16b7ee4d5f6627a6db78a**: C1–C4 and G2 PASS. Four packages build; 77 unit tests (0 errors/failures/skips), 26 analytic checks, 37 ROS checks and 19 health-gate checks. Paired 25.44s physical runs have exactly equal full qpos/qvel; settled heading 102.05°. Quantized wall error across physical phases: worst phase P95 0.01423m, max 0.05772m; scan compute P95 1.285ms (<50ms). Actual ROS: 307 scans and 307 raw clouds, 10Hz simulation frequency, settled heading 94.59°, wall P95 0.008736m/max 0.04175m; scan compute P95 1.817ms, median RTF ~1.0. No self hits. All own processes exited, dedicated port is unused, both upstream SHAs remain clean, and the user diff hash is unchanged. Final source/test-script hashes are recorded in source-audit.json. Failed development attempts remain in history/evidence and do not count as passing runs.

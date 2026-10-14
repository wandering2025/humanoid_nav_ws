# W07 static scenes and robot description evidence

Only W07 is implemented here. No LiDAR, SLAM, Nav2 or moving obstacle logic is claimed.

Layouts/configs: `src/t1_mujoco/worlds/*.yaml`; conservative footprint and RViz: `src/t1_description/config/{footprint.yaml,scene.rviz}`. Matching external 23DOF URDF is referenced via `t1_bringup/description.py`. Sources/license boundary: sources.md; dimensions/envelope/FK limitations: geometry.md; four `*-top.png` are metric config-derived illustrations, not navigation results. `mujoco-static-top.png` and `mujoco-robot.png` render an actual recorded ROS truth/joint snapshot in an independent visualization data object; RViz screenshots are actual windows from physical simulation.

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
source scripts/env.sh
/usr/bin/colcon build --base-paths src
source scripts/env.sh
/usr/bin/colcon test --base-paths src
/usr/bin/colcon test-result --verbose
$T1_VENV/bin/python docs/evidence/W07/check_scenes.py
$T1_VENV/bin/python docs/evidence/W07/check_description.py
$T1_VENV/bin/python docs/evidence/W07/check_motion.py
MPLCONFIGDIR=/tmp/w07-matplotlib /usr/bin/python3 docs/evidence/W07/draw_scenes.py
# Actual launch/ROS/RViz; script owns and stops a dedicated discovery server.
export T1_LOCAL_DDS=1 T1_DDS_PORT=42117 ROS_DOMAIN_ID=77
export ROS_LOG_DIR=$PWD/run_data/w07_ros_logs
source scripts/env.sh
$T1_VENV/bin/python docs/evidence/W07/verify_ros.py
MUJOCO_GL=glfw $T1_VENV/bin/python docs/evidence/W07/render_snapshot.py
```

This host requires permitted local sockets/X11 for integration and GLFW. Simulation Python is always explicit venv; only standalone plotting uses preinstalled system matplotlib. The description replay reads the preserved ignored W05 C3 raw trace and verifies its hash/source manifest; if missing, regenerate W05 C3 under its documented frozen configuration before replay, rather than inventing data. No dependencies were installed or upgraded.

Manual named-scene launch (use a separate dedicated discovery-server terminal as documented in W06 if needed):

```bash
ros2 launch t1_bringup simulation.launch.py scene:=static_world rviz:=true
# alternatives: calibration_world, dynamic_crossing_world, stress_test_world
# scene:=/absolute/path/layout.yaml or scene:=/absolute/path/model.xml also accepted
# Ctrl+C stops this launch; no navigation or motion command is auto-generated.
```

`scene-load.json`: four models, named geom/body uniqueness, index mapping, exact original physics comparison. `description-envelope.json`: all-body FK, raw mesh visual scale, 10 representative W05 trajectory replays, conservative body sweep. `scene-motion.json`: physical forward/walking turn/in-place turn/stop in all four worlds, full qpos/qvel equality to upstream, contacts sampled every 0.002 s substep. `ros-rviz.json`: actual launch, live RSP vs independent FK, simulation time, frame ownership, interpreter and orderly exits. Tests/build text: 68 tests and four packages. `failed-attempts.json` preserves development failures and fixes; raw logs and state arrays are ignored. `source-audit.json` binds tested code to final source; `process-audit.json` records own process cleanup.

W07 geometry width is not a braking safety margin. W10 must derive safety zones, W11 must verify goal/path feasibility and the W05 time/yaw limits. W12 implements motion using region/slot metadata; W07 does not write future trajectories or robot routes.

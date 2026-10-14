# W09 SLAM mapping and saved-map localization evidence

Final build, tests and mapping regression target `5d5ce34be030a7c060ebdd56c43023be7eca7cc1`. Physical map generation targets `2cf9496729f6002fdceda90663000b2550ab126d`; three independent AMCL trials target `680fb71c8f0739bbd705b47faf9a3df33ef93606`. The later source changes add provenance logging and documentation; physics, LiDAR, SLAM/AMCL parameters, and map files used in these trials are unchanged.

All C1–C4 acceptance passed. No W10/W11 work is started.

- Four packages build; 85 unit tests, zero errors/failures/skips.
- Mapping: six predetermined inspection legs through both corridors and B, 24.13m actual physical path, 0.0774m loop endpoint error. 110 map updates at median 1.0s simulation interval.
- Map: 0.05m resolution, 201×201 cells, A/B and 54 corridor samples free, wall surface error P95/max 0.059m, visible surface completeness P95 0.0545m.
- Actual RMW publisher identity: only slam_toolbox publishes map→odom in mapping; only AMCL in each localization run. The simulator owns clock/odom and never publishes map→odom.
- AMCL: map_server then AMCL lifecycle active; three full independent starts load the identical YAML/image checksum and receive exactly one approximate initialpose.

| Initial map pose (m, m, rad) | Convergence (sim s) | Final position error (m) | Final yaw error (deg) | Scan-to-map P95 (m) |
| --- | --- | --- | --- | --- |
| [0.2, -0.15, 0.15] | 1.54 | 0.00632 | 0.3935 | 0.04953 |
| [-0.2, 0.15, -0.2] | 2.74 | 0.04823 | 0.1342 | 0.06705 |
| [0.1, 0.1, 0.3] | 2.54 | 0.02844 | 0.5019 | 0.04103 |

Predetermined limits are in map metadata: 25 simulation seconds, position≤0.25m, yaw≤10°, scan distance P95≤0.15m. Each last three pose updates and all later updates after reported convergence satisfy the pose limit. Scan alignment uses 24 successful TF queries at actual scan stamps, 8640 endpoints. Reported errors compare the ideal odometry/world horizontal projection against AMCL using T_world_map; truth is evaluation only.

## Reproduce

Use this machine’s frozen paths and environment, no installs/upgrades:

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42119 ROS_DOMAIN_ID=29
export ROS_LOG_DIR="$PWD/docs/evidence/W09/raw/ros-log"
source scripts/env.sh
/usr/bin/colcon build --symlink-install --base-paths src
source scripts/env.sh
/usr/bin/colcon test --base-paths src
/usr/bin/colcon test-result --all --verbose
cmake -S docs/evidence/W09 -B docs/evidence/W09/raw/authority-build -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build docs/evidence/W09/raw/authority-build
"$T1_VENV/bin/python" docs/evidence/W09/run_checks.py c1
"$T1_VENV/bin/python" docs/evidence/W09/check_localization.py
/usr/bin/python3 docs/evidence/W09/final_checks.py
```

The probes own their discovery server and launch processes. Do not run a second server on port42119. A tool sandbox needs localhost DDS access and host PID visibility; normal project terminals already have these. ROS logs and large raw snapshots/serialized pose graph stay ignored. The C++ observer is needed because this installed rclpy MessageInfo omits publisher_gid.

For user-facing live commands see the root README and docs/architecture/mapping_localization.md. To repeat mapping generation, `run_checks.py mapping` executes the predetermined inspection route and writes the tracked `src/t1_navigation/maps/static_world.yaml/.pgm`; then run `MPLCONFIGDIR=/tmp/t1-w09-matplotlib /usr/bin/python3 docs/evidence/W09/analyze_map.py` to update quality/provenance. This regeneration changes versioned map/evidence files: inspect the Git diff before committing. The plotting tool uses already installed system Pillow/SciPy/Matplotlib; simulation always uses the configured venv.

## Artifacts and limits

- `mapping.json`, `mapping-leg-1.png`, `mapping-leg-3.png`, `mapping-final.png` preserve actual building progress, TF identities and map updates. `map-route.png` plots the saved SLAM occupied cells and measured walking path.
- `localization-{0,1,2}.json`, `localization.json`, pose-error/trajectory CSV and `localization.png` preserve initialization, lifecycle, scan alignment and restart evidence. Images were visually inspected.
- `c1-checkpoint.json` is the first passing C1; `c1.json` is final regression. `final-verification.json` contains source hashes, map/scene binding, frozen repository audit, unchanged user-diff hash and stopped processes/closed DDS port.
- Failed attempts (`c1-*-attempt.json`, `c1-metadata-inspect.json`, `localization-attempt1.json`) remain FAIL. Sandbox socket denial, missing Python message GID and NumPy bool JSON serialization were corrected without changing the physical policy or installed dependencies.
- `static_world.meta.yaml` binds map checksum, scene checksum, source SHA, sensor mode and world/map transform. The optional pose graph is in ignored `raw/static_world.*` for continuation/debug; AMCL needs only the tracked YAML/image.

Ideal truth odometry, truth attitude compensation, noiseless instantaneous multibeam LiDAR and planar static world are explicit assumptions. Results do not establish drift/noise robustness or real-robot localization. Back faces not observed by LiDAR remain unknown. The route is controlled inspection replay, with W05 command mapping and W04 watchdog; it is not autonomous Nav2 navigation or the W10 full safety chain. No remaining own ROS/simulator processes; both frozen external repositories remain clean.

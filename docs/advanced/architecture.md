# Advanced SLAM and Navigation Demo v0.2

Status: IN_PROGRESS, not a release. This independently authorized phase supersedes the old single-W/keep-branch restriction only for v0.2. Stable v0.1.0 remains untouched in its original worktree.

The observed v0.1 capabilities are physical T1 locomotion, compensated LiDAR, separate SLAM mapping, saved-map AMCL/Nav2, reactive dynamic-obstacle waiting and safe shutdown. Existing mapping is a scripted command experiment, not autonomous exploration.

## Decisions

Use guided autonomous mapping (predefined map-frame waypoints executed by Nav2), not frontier exploration. Start with a blank SLAM Toolbox session. Grow the map while walking through the west room, wide north gallery and east room around an occluding partition; return to A, save the real SLAM OccupancyGrid and plan a complete nonstraight route to B. Keep SLAM as the only map→odom authority throughout; no AMCL handover, prior map, scene rasterization or truth correction. NavFn refuses unknown space. Navigation consumes only scan/map/TF; evaluation may read truth.

A=(0,0), B=(14,0). Inner bounds [-4,18]×[-4,12]; partition x=[7,8], y=[-4,4.5]. The 7.5m northern passage preserves the 2.05m planning radius, 1.65m stop and 1.95m slowdown radii. A/B annotations stay in user_scn/RViz, never physics or sensors.

A staged mocap obstacle is explicitly enabled only after mapping, then approaches the final route, holds and withdraws. The event source may use robot pose to trigger the experiment; neither obstacle truth nor future motion is exposed to the planner. Reuse the unchanged locomotion, safety, command adapter, goal verification, production renderer lifecycle and GPU monitor. Recording stays independent of demo runtime.

## Verification gates

CPU build/contracts and stable-source regression → autonomous headless full mapping/navigation → production minimal standing renderer → full GUI and RViz → independent real-time recording and CPU decode/visual review → provenance/cleanup/port audit. Every failed attempt is retained. No v0.2 tag until all presentation gates pass.

Known inherited limits: ideal truth odometry, truth attitude compensation, finite rays, flat terrain, gait deadband/rebound, kinematic obstacles, reactive navigation without prediction, frozen WSLg/NVIDIA graphics path, existing strictly bounded DDS teardown exception. W13 remains FAILED 36/40; this phase is not a reliability sweep.

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

## Observed response and verification refinement

Headless02 completed physical mapping/return/navigation with zero contacts/falls, but its original result is FAILED because the first verifier accepted only protective waiting. The robot actually replanned around the approaching obstacle before Collision Monitor needed to stop it. Original failed result is retained. User explicitly permits waiting, detouring or replanning. From the next candidate, accept either >=0.5s measured protective stationarity OR a replanned path deviating >=0.5m plus actual robot deviation >=0.25m and travel >=0.5m during the active obstacle. Both require actual LiDAR hits, costmap marking, obstacle clearance, arrival and all unchanged safety/stop/cleanup gates. These are behavior evidence criteria, not changes to safety thresholds.

Map/TF provenance is now collected using ROS publisher GIDs; /map and map→odom must both be exclusively SLAM Toolbox. Saved map sidecars bind the scene, SLAM parameters, code SHA, run ID and observed map→odom transform. Only evaluation reads these sidecars.

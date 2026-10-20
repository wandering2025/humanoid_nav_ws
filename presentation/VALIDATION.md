# Validation

## Local reliability rehearsal (2026-10-20)

**FAILED — live-demo reliability has not been accepted.** The user requested local load tests first because Google Meet sharing was unavailable. The test load continuously read both real windows and encoded1920×1080 H264 at15fps on CPU. Unique window captures were about6–7fps, recorded separately from encoded frames. This is a local proxy, not a Google Meet/network/recipient test.

Current online implementation: `5f1d1d3837551d5d68e0ed899c709cbef508326d`. The one-line graphical command remains unchanged. The frozen walking controller,2ms physics,20ms policy,10Hz LiDAR,target_rtf1 and0.3s safety timeout were preserved; all current-version run source hashes match.

| Current-version test | Result |
|---|---|
| GUI + capture/encode +60s arrival hold, started from /tmp | PASS; all18 independent audits;230 active obstacle scan frames,34 marked costmaps,18.22s wait,4.132m resumption,0.103683m final-window error; all exits0 |
| Next loaded GUI mission | FAIL; actual0.374138s scan gap triggered the original0.3s timeout and latched safe stop; batch stopped |
| Additional loaded GUI profiling mission | FAIL; reached B at0.058079m and passed dynamic response, but DDS exited−11 after all clients disconnected |
| Deliberate startup / walking interruptions | Both PASS for safe cancellation and cleanup; missions correctly remain FAILED, not successful arrivals |
| Five ordinary GUI repetitions / remaining three planned load repetitions | NOT_RUN on final implementation after fail-fast rejection; earlier versions are historical |
| CPU / build |260 passed,1 known historical v0.1 source-identity assertion deselected; four ROS packages built with system colcon |
| Actual Google Meet screen sharing | NOT_RUN |

Across this investigation there were20 robot/GUI attempts on multiple implementations:8 successful and6 failed full-mission attempts,2 failed diagnostic runs, and4 successful expected-cancellation tests. This is **not a final-version success rate**. All20 recorded physical safe stop, zero environment/dynamic/non-foot contacts and falls, and complete owned-process/port recovery. Failed missions remain failed despite safe recovery.

Verified changes improve the controlled obstacle encounter trigger, exact-time evaluator evidence, and Ctrl+C/hold exit handling. Closing the evaluator before DDS is retained, but it did not eliminate the native crash. A separate robot-free reproducer captured the same libfastrtps2.14.7 TCP receive-thread crash. Waiting for receive threads to drain still failed on trial13; UDP discovery delivered no cross-process messages. Neither experiment was adopted. A synthetic Python thread-switch experiment did not establish the real scan-delay cause, so no scheduler or host power settings were changed.

The two unresolved items are physical scan latency under load and DDS native teardown. The preregistered5 ordinary GUI +5 proxy-load (including hold) +2 cancellation acceptance matrix must restart after proven remedies. No failure exception, reduced load or relaxed safety threshold was used to pass it. The earlier success below remains valid for its recorded version, but does not establish current live-demo reliability.

[Local test summary](../docs/evidence/W16/reliability/local-load-summary.json) · [Every robot attempt](../docs/evidence/W16/reliability/history.json) · [Physical latency evidence](../docs/evidence/W16/reliability/physical-latency-diagnosis.json) · [Native DDS diagnosis](../docs/evidence/W16/reliability/dds-native-diagnosis.json) · [Isolated reproducer](../docs/evidence/W16/reliability/reproduce_dds.py) · [Capture decoding checks](../docs/evidence/W16/reliability/capture-verification.json).

## W16 exploration with graphical simulation and dynamic obstacles

**Verified GUI run: SUCCESS. User test accepted on 2026-10-20 (Europe/Berlin).** The user reported “我测试过了，看起来很棒” and requested this presentation update. That feedback is separate from the measured runs below; no additional user-run metrics are inferred. That earlier presentation update changed documentation only and did not rerun simulation. Its command, link and configuration checks are recorded in the [documentation verification](../docs/evidence/W16/presentation-verification.json).

Online implementation: `7fa9eeb1daed42cb74cd8067f547c46b4c643920`. Final evidence/documentation: `06efd73da967abf822a63a0183637fb91440a8c0`. The task specifies B=(11.0, 0.5) **relative to the actual starting frame A**.

| Check | Recorded result |
|---|---|
| Build / CPU validation | Four ROS packages built with system colcon; 171 CPU tests passed |
| Complete GUI dynamic run | SUCCESS; MuJoCo + RViz, seven decisions (six exploration targets, then B) |
| Real obstacle / sensor response | Collidable mocap body travelled 6.8m; 83 active scan-hit frames and 37 local-costmap marking frames |
| Physical response | Stationary wait 17.72s; resumed 4.132m after the obstacle departed |
| Arrival | Maximum position error over the final two seconds: 0.095193m; physically settled |
| Continuous mapping | SLAM remained active; all 649 sampled points inside the former blockage were free in the final actual SLAM map |
| Same-code static control | SUCCESS with `--headless --static`; no dynamic body, eight decisions, final-window maximum error 0.062906m |
| Path audit | All 7 GUI + 8 control paths independently replayed against their contemporaneous observed costmaps: known and reachable |
| Safety / cleanup, both runs | Zero static, dynamic and non-foot ground contacts; no falls; physical safe stop, all process exits 0, no residual processes, DDS port released |
| GUI / independent audits | Six actual window screenshots; NVIDIA rendering; no GPU/host fault; 17 GUI and 15 final audit checks passed |

Measured GUI run: `20261019T220018Z_exploration_f55dc7ff-76c0-4473-af02-96b4fc3657a1`, tested SHA `0e2b302721fc11324f2392a20f6c892e7e7eac86`.

Static control: `20261019T220515Z_exploration_43ca27a8-1688-4c8e-b631-e6e1b2704ee3`, tested SHA `e91599bb27e1ba7076bc6f2e82e7072a9c442a28`. All `src/` and `scripts/` files were byte-identical between these two runs; the evidence records each full run version.

[Run and path audit](../docs/evidence/W16/gui.json) · [SLAM map and actual trajectory](../docs/evidence/W16/gui.png) · [Dynamic-response audit](../docs/evidence/W16/gui-dynamic-audit.json) · [Response plot](../docs/evidence/W16/gui-dynamic-audit.png) · [Static control](../docs/evidence/W16/gui-static-control.json) · [Final verification](../docs/evidence/W16/gui-final-verification.json).

Actual window evidence: [dynamic MuJoCo](../docs/evidence/W16/gui-dynamic-mujoco.png), [dynamic RViz](../docs/evidence/W16/gui-dynamic-rviz.png), [arrived MuJoCo](../docs/evidence/W16/gui-arrived-mujoco.png), [arrived RViz](../docs/evidence/W16/gui-arrived-rviz.png). The arrived RViz screenshot retains an earlier “Waiting” label; the physical trace, settling checks and final result establish arrival.

Three GUI development failures remain in the [full W16 evidence](../docs/evidence/W16/README.md): mapping backlog/TF extrapolation, a stale raw scan that correctly latched safety stop, and expiration of the former local waiting budget. The successful GUI run used Nav2 waiting and resumption; the added `known_free_reobservation` branch has CPU coverage but was not exercised in that successful physical run. Those earlier runs are not the reliability campaign above; the historical W13 FAILED result and occasional lifecycle startup timeouts remain recorded.

The robot uses physical policy-driven walking, ideal odometry and limited LiDAR with truth attitude compensation. The obstacle is a collidable, non-pushable kinematic body. Observation-based filtering affects only SLAM input; Nav2 and Collision Monitor still receive raw LiDAR. This does not establish general dynamic SLAM, motion prediction, real-robot readiness or success in arbitrary layouts. No W16 video was recorded. Older evidence fields marked `USER_TEST_PENDING` describe the pre-feedback audit and are preserved as historical data.

## Historical guided-demo validation

The following results belong to `run_advanced_demo.sh`, which uses preset mapping waypoints and absolute map goals. They are retained for the legacy command/video and are not W16 measurements.

PASS. Implementation `d7b4c1742776dfae9019f3c25241f7c9c51f04ea`; evidence `342889126827b8bb4844d229ea2831a93bc5783d`. No runtime source changes between the runs below; the edited-goal run intentionally had only `presentation/goal.yaml` locally modified. The file was restored to B=(12,0) at that historical checkpoint; subsequent user edits are preserved.

| Test | Result |
|---|---|
| Four ROS packages, system colcon | PASS |
| Complete CPU suite | 214 passed; 1 existing historical v0.1-only source-shape assertion deselected |
| Focused interface/contracts | 27 passed: inclusive corners, finite numeric values, invalid/duplicate keys, CRLF exact hash, snapshot consistency and render-only marker/LiDAR isolation |
| Actual CLI from outside workspace | Changed valid B read correctly; out-of-range B rejected; no simulation or runtime directory created |
| Default B=(12,0), full headless | SUCCESS; run `20261016T102558Z_advanced_bb6bc3ae-7927-4029-af26-b8a1082e6f62` |
| Edited B=(11,0.5), standing GUI | SUCCESS; NVIDIA MuJoCo/RViz rendering, no movement or new graphics fault |
| Edited B=(11,0.5), complete GUI | SUCCESS; run `20261016T103421Z_advanced_3ef55d11-4de2-4e6d-baa7-3f685541e579` |
| Real RViz marker and NavFn target | 810 B-marker observations at (11,0.5), 1 acquired-map overview target checks, no mismatch |
| Final changed-goal arrival | 0.10166m position / 0.03271rad heading error; original 0.20m/0.20rad arrival checks |
| Dynamic interference | Default wait 16.00s; edited-goal wait 14.10s, then continues 6.74m and arrives |
| Safety and cleanup, both complete runs | Static contacts 0, dynamic contacts 0, non-foot ground contacts 0, falls 0; physical safe stop, all infrastructure exits 0, no residual processes, DDS 42127 released |
| Stable releases | Original worktree clean, v0.1 source/entry preserved; all three existing release tags unchanged; no push |

Both runs bind the exact YAML text/hash to the simulator manifest and navigation result. Their goal, derived approach, real overview endpoint and arrival verification agree. The edited run's actual MuJoCo/RViz final screenshots were visually inspected. See [evidence](../docs/evidence/presentation-interface/README.md).

This validates the default and an edited boundary configuration, plus geometry checks across all rectangle corners. It does not claim statistically measured reliability for every coordinate. The mapping route remains guided; ideal odometry and truth attitude compensation remain. SLAM corrections pause during dynamic navigation while live LiDAR obstacle avoidance continues.

The existing 161.5s published backup video is unchanged and uses B=(12,0). No replacement video or new release tag was created for this presentation-kit task.

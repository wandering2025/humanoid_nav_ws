# W15 final integration review

Final disposition: **FINAL_RELEASE_BLOCKED**. No release tag or successful final video is issued. Task 2 is not presentation-ready. W15 ends at this explicit blocked decision; there is no W16 or implied continuation.

The release candidate adds render-only A/B markers, restrained materials, a full-route camera, a clearer RViz configuration, automatic final-view cleanup, and a real-time CPU H264 dual-window capture tool. The new visuals have not passed a full graphical rehearsal. Physics, map bytes, LiDAR, obstacle trajectories, navigation/controller configuration, and all safety thresholds remain frozen.

The first static rendering check failed on a screenshot readiness race and was retained. After that bounded test-script fix, the second check exited −11. The kernel records a general protection fault in NVIDIA's user-mode `libnvwgf2umx.so`, on `python:gdrv0`, with the crash captured for renderer PID68872. Windows event queries did not match a new display/LiveKernelEvent, which does not prove host graphics safety. GPU/GUI/capture expansion stopped and the halt remains latched. This failure is outside the approved DDS teardown exception.

The new static checker omitted waiting for MuJoCo's daemon rendering thread after close; the production simulator already does this. An offline correction now mirrors the production close/join sequence. This is a plausible lifecycle contributor, not a proven driver root cause or a validated fix. No additional graphical run was performed after the fault.

| Requirement | Actual implementation | Existing evidence | W15 independent evidence / limit |
|---|---|---|---|
| R1 Booster T1 physical walking | Frozen official t1_walk → joint targets → PD → 500 Hz mj_step | W04/W05 physical traces | Source audit; CPU static/blocking traces; no base-pose locomotion |
| R2 LiDAR perception | Body-fixed 9×360 mj_multiRay rays, projection to /scan; full robot excluded | W08 ray/TF tests; W12 dynamic hit samples | Unchanged sensor/config hashes; regression tests; dynamic check |
| R3 SLAM/localization | SLAM Toolbox-generated saved map; AMCL owns map→odom in live navigation | W09/W12 maps and generation metadata | Map provenance tests and single-owner TF audit; live SLAM is a separate mode |
| R4 A→B / dynamic obstacle | NavFn + RotationShim/MPPI; reactive scan/costmap avoidance; kinematic obstacle | W13 formal-v1 36/40, with all failures retained | S01/B01 independent checks; final default GUI NOT_RUN |
| R5 stops/recovery | gate → smoother → Collision Monitor → only-reducing bridge; dual-clock watchdog | W10 fault matrix | source_timeout stop/latch/explicit-rearm check; safety thresholds unchanged |
| R6 reproducible demo | One entrypoint, explicit venv, readiness/stop/owned-process cleanup | W14 two prior runs, bounded DDS warning | CPU preflight/build pass; final GUI blocked by new minimum-stage graphics failure |
| R7 provenance | Same release branch, full SHAs, immutable old evidence, ignored raw logs | W01–W14 records | W15 raw hashes, recorded failure/CPU results, clean committed handoff |
| R8 presentation/backup | Operation guide and workflow/limitations | W14 presentation outline | Revised guidance; no valid successful release video, so FAIL |

W13 remains **FAILED**: static 10/10, crossing 8/10, blocking 8/10, sudden approach 10/10, total 36/40; collisions 0, falls 0. This review neither replaces that campaign nor upgrades its reliability verdict.

Allowed simplifications remain explicit: ideal truth odometry, truth attitude compensation, finite instantaneous beams and default zero noise, flat wide corridors, kinematic mocap obstacles that cannot be pushed, reactive avoidance without obstacle tracking/prediction, gait deadband and yaw rebound, nondeterministic scheduling, and sampled rather than continuous clearance. The protections are not certified safety systems. WSLg OpenGL uses Mesa/D3D12 on the selected NVIDIA adapter; native Linux direct rendering and host fault isolation are not claimed.

Release gate is closed specifically because the new render path has a genuine graphics-library exit failure and there is no verified full default run or successful final recording. Lowering safety criteria, treating the MuJoCo crash as a DDS warning, deleting failed evidence, or using a previous-version video would not satisfy this gate.

## Versions and actual verification

Branch `release/w13-formal-v1-demo`; start `e916d0db2a10b1728c1421264336564884e3d7ea` clean. Final implementation/tests `f09f7d80169033edf3486903db10a7290535507f`; final evidence `b2c0be2f6ce2b331993a556184fbe0f4e87753ce`. No history rewrite, push or release tag. Raw recordings/traces/build artifacts remain ignored.

| Check | Actual result | Evidence |
|---|---|---|
| System colcon | Four packages built, 2.76s | W15/raw/build.log |
| Offline regressions | 186 passed before graphical checks | W15/raw/unit-tests.log |
| Final affected tests | 55 passed; outdated unlocked-profile assertion failure retained separately | W15/raw/final-unit-tests-02.log |
| CPU preflight / default GUI lock | exit0 / exit3 before GPU access | W15/raw/final-preflight.log, final-gui-refusal.log |
| Static S01 | SUCCESS, .106857463m / .603794073deg, zero contacts/falls | W15/review-summary.json |
| Blocking B01 | FAIL_TIMEOUT at120s; approach PATIENCE_EXCEEDED104, final goal CANCELED; zero contacts/falls | Same summary and immutable raw result |
| Source timeout / recovery | PASS; zero within.30sim s; max travel after zero.184094256m; stand/latch/rearm | Same summary |
| Graph / physics / sensors | Original clock/TF/command checks pass in CPU cases; all99 nonvisual locked files unchanged | verification.json and review-summary.json |
| Final GUI / successful recording | NOT_RUN / MISSING; minimum stage graphical FAIL prevents load expansion | graphics-blocker.json |
| Cleanup | No project/review/capture processes; all6 ports bindable | final-process-port-audit.json |

The source-timeout fixture startup failure was corrected and retained; no robot ran in that first attempt. The legacy CPU harness does not persist DDS's exact exit status, so it cannot justify DDS exception approval. Two diagnostic videos decode fully, but contain no successful task; their paths, byte counts and SHA256 are in diagnostic-video-validation.json. The full release recording path/hash and successful GUI run ID are N/A.

Final acceptance: C1 source/contract audit PASS; C2 independent tests FAIL (B01 and graphics); C3 Git/dependency/evidence audit PASS; C4 release BLOCKED. This is a closed blocked handoff, with no automatic next work item.

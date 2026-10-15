# W15 final independent review

**DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**. The recovered, fixed U01 presentation starts with `./scripts/run_demo.sh`, navigates physically from A to B, safely waits for an approaching dynamic obstacle, resumes, stops, and cleans up. The final recorded run is SUCCESS with zero contacts/falls and all component exits 0. This is a demonstration release, not a production reliability claim.

The original W15 FINAL_RELEASE_BLOCKED decision remains valid for that failed candidate and is preserved in [the original review](w15-original-blocked-review.md), [original manifest](../demo/w15-original-blocked-manifest.md), the original branch and raw evidence. The user's subsequent authorization resumed W15 on a new branch from the verified W14 final commit. No W16 was created.

| Version | Full SHA / identifier |
|---|---|
| Recovery branch | release/w15-controlled-recovery |
| W14 recovery base | e916d0db2a10b1728c1421264336564884e3d7ea |
| Preserved original W15 branch HEAD | 37a765c3677b258d04e6bb413570088b1d77a482 |
| Final implementation/publication status | 21df4b116f0e987690c6288e400c7fa0e45a65b5 |
| Final evidence | 945eb32bd774a206743ab6d06432c6024bb0504a |
| Actual recorded clean candidate | 127a1e7672f72616de728c84f41701669a41f1c9 |
| Final run | 20261015T155946Z_demo_U01_79a6bb62-5b16-427f-986d-eb23df9a93c1 |
| Release tag | demo-v0.1.0, on final handoff commit |

The recorded candidate and release differ only in help/profile informational status and documentation/evidence. The [equivalence audit](../evidence/W15/recovery/publication-equivalence.json) checks every source/script, the shell body and profile fields. Physical behavior, navigation, safety, rendering lifecycle and recorder are identical.

| Requirement | Implementation and evidence | Assessment / boundary |
|---|---|---|
| R1 humanoid physical locomotion | Official Booster T1 policy → joint PD → MuJoCo2 ms dynamics; frozen checkpoint/model | PASS; no base-pose locomotion or teleoperation |
| R2 LiDAR | Actual9 × 360 rays, body-attitude compensation, real dynamic geometry hits; final 491 hit frames | PASS; finite/noiseless default beams, truth attitude |
| R3 SLAM/localization | SLAM Toolbox-produced saved map; AMCL solely owns map→odom in navigation | PASS; current demo uses localization, separate mapping mode; ideal truth odom disclosed |
| R4 A→B / dynamic avoidance | U01 fixed 4000; dynamicPASS, physical stop/resume, verified goal; independent B01 control demonstrated a real detour | PASS for presentation; U01 is not persistent-blocker detour; historical failures retained |
| R5 safe stop/recovery | Original gate/smoother/Collision Monitor/only-reducing bridge; finalACK and physical stationarity | PASS; thresholds unchanged; prior W15 source_timeout11 checks at 3bacd05595fb5a234d0ebc019604c82895c43f14 reused because those sources are unchanged |
| R6 one-command operation | Fresh stripped terminal environment; explicit configured venv; readiness, safe shutdown, final port check | PASS on frozen Ubuntu24.04/WSLg/NVIDIA4050 host; not an installer for arbitrary systems |
| R7 provenance | Preserved original branch/history, fixed upstreams, clean checkpoints;699 new raw hashes and original 155 verified | PASS; raw movies/traces remain ignored, hashes tracked |
| R8 presentation and recording | A/B MuJoCo and RViz labels, simple colors/headlight, presenter docs, verified H264 movie | PASS; single-window8 fps footage, separate RViz screenshots, no audio |

Recovery deliberately removed the failed standalone static viewer checker, its missing thread-join path, 107 decorative geoms/grid, full-route camera, ctypes window manipulation and automatic dual capture. The production W14 close/join lifecycle stayed intact. Four static user_scn geoms and render-copy colors do not enter MjModel, rays, contacts, maps, costmaps or navigation predicates. The only default lifecycle change is an 8-second final view after the existing safe stop, then the existing cleanup.

## Verification and failures

- Four packages using system colcon built; 186 offline regressions passed under CPU device denial; 55 affected tests passed after the disk-size fix and capture preparation. CPU preflight on the final implementation passed.
- Source audit: 99 locked nonvisual files unchanged, W13/W14 evidence and records untouched, external deploy/assets fixed and clean. Dynamics/contact/ray equivalence checked; simulator diff only adds viewing hooks.
- B01 prior failure used unchanged motion/map/safety inputs. One preregistered control at fdd85108ed1dd19edb123388da042d9240f641b6 succeeded in 55.14 simulated seconds, with real detour and no collision/fall. The prior timeout was not reproduced; this does not establish a reliability rate or prove a unique timing root cause.
- One production minimal GUI stage succeeded: 30 simulated seconds standing, safe stop, all exits 0 and port released. Its screenshot exposed a display-only cylinder half-height error; the single input was corrected and CPU checked. No extra minimal GUI was run.
- Full rehearsal `20261015T154938Z_demo_U01_0fd7462e-1531-4919-8331-345d97ac0f06` succeeded, dynamicPASS, all exits 0.
- Recording attempt01 `20261015T155453Z_demo_U01_a8b154bb-ac95-4cff-b0a2-15a18addbdc8` navigated and stopped successfully. DDS exited−11 only during expected teardown; all original 14 gates passed, yielding SUCCESS_WITH_WARNING while raw FAILED remains. Capture failed before drawable readiness with 0 frames. Later original screenshots succeeded; MuJoCo/RViz exits 0 and host checks were clear. This was a capture-readiness error, not a new GPU crash.
- The independent recorder was corrected to wait for the demo's complete ready PNGs. The replacement final recording run succeeded with all exits 0. This extra run is explicitly recorded, not concealed as a first-attempt success; no repeated NVIDIA failure path or reliability sweep was used.
- Final navigation verified in 46.4 simulated seconds; settled error 0.123586349 m / 2.216213678°; protective hold 7.08 s; sampled body minimum 1.444234181 m; contacts 0, falls 0; safe-stop ACK/stationary true; no remaining tasks and seven ports bindable.
- GPU during final recording 49–64°C, 998–1390 MiB, 5–63% utilization; no new rendering crash or host display event. Repeated pre-existing dxg adapter/feature-query diagnostics remain disclosed. Monitoring does not provide host isolation.

## Recording and limits

[Final movie](../evidence/W15/raw/recovery/final-recording-02/demo.mp4): 1896620 bytes; SHA256 `9980c060692af05c26f6ed83ac2a3a5b5dd88935a4cc72de8fdd1966470e0df0`. 1024×736 / 8 fps, 77.125 s, no audio. 418 real PNG captures retain timestamps and hashes; latest frames are repeated to preserve 69.125 seconds wall time without speed-up. An explicitly labeled 8-second post-run evidence card follows. 617/617 frames decoded using CPU; encoded frames 0/160/320/520/600 visually reviewed. No new graphical video player was launched.

W13 remains **FAILED**: static 10/10, crossing 8/10, blocking 8/10, sudden approach 10/10, total 36/40; collisions0, falls 0. Ideal odometry, truth attitude, finite/noiseless beams, wide flat corridors, kinematic mocap obstacles, reactive navigation without prediction, gait deadband/yaw rebound and scheduling variability remain. The old NVIDIA/WSLg and RViz failures are not claimed to have a proven library root-cause fix. The DDS teardown defect remains under the unchanged 14-condition exception; any runtime failure, unsafe stop, other component failure or host graphics fault is excluded.

Final checkpoints: C1 PASS, C2 PASS within the user-authorized demonstration scope with all failed attempts retained, C3 PASS, C4 PASS once final documentation/handoff/tag are committed. Task2 development ends with this W15 release; no undefined follow-up or W16 is a completion condition.

[Release manifest](../demo/release-manifest.md) · [Recovery evidence index](../evidence/W15/recovery/README.md) · [Presenter guide](../demo/live-demo-runbook.md).

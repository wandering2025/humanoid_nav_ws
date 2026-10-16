# Validation

PASS. Implementation `d7b4c1742776dfae9019f3c25241f7c9c51f04ea`; evidence `342889126827b8bb4844d229ea2831a93bc5783d`. No runtime source changes between the runs below; the edited-goal run intentionally has only `presentation/goal.yaml` locally modified. The supplied file has been restored to B=(12,0).

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

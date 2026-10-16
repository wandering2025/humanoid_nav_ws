# Presentation-kit and real B-point interface evidence

Implementation `d7b4c1742776dfae9019f3c25241f7c9c51f04ea`. Each launch reads presentation/goal.yaml; the exact source/hash is snapshotted and shared by navigation, actual NavFn overview, RViz, MuJoCo render-only B, simulator manifest and arrival verification.

- [Default full run](default-run.json): B=(12,0), complete headless SUCCESS.
- [Edited-goal full GUI](changed-run.json): only goal.yaml changed to B=(11,0.5); SUCCESS with matching navigation/simulator snapshot, derived approach, real overview endpoint and arrival. Actual MuJoCo/RViz final screenshots inspected.
- [Readonly marker audit](changed-marker-audit.json):810 correct B marker samples and actual overview endpoint(11,0.5), no mismatches. The observer published no motion commands.
- [Changed-goal minimum graphics](minimal-changed-goal.json): zero policy, actual NVIDIA MuJoCo/RViz rendering, stop/cleanup/port PASS before full GPU workload.
- [CPU/CLI/build](checks.json):214 full CPU tests pass,1 existing historical v0.1-only source-shape assertion deselected;27 focused pass;4 ROS packages built. Real CLI invoked outside workspace reads changed values, rejects invalid values and creates no runtime directories.
- [Source/stable/cleanup audit](source-audit.json): original stable worktree and all three release tags untouched; simulator delta is only advanced-goal metadata and render-only argument; original gait/adapter/safety/controller/scene/events/GPU monitor unchanged. Default goal file restored byte-for-byte; no remaining project processes;42127 bind PASS.
- [Raw SHA256 index](raw-index.json):195 runtime/analysis files across both full missions and the graphical standing check. Raw data remain ignored; no previous failures deleted or overwritten. Analysis helpers are in tools/.

All three runtime checks have zero static/dynamic/non-foot contacts and zero falls; all infrastructure exits0 and cleanup/port release pass. No new video/tag/push/driver or dependency changes. Existing published video remains the default B=(12,0) recording. [Operator kit](../../../presentation/README.md) and [tested configurations](../../../presentation/VALIDATION.md).

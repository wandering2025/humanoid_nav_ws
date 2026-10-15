# Advanced navigation optimization evidence — 2026-10-16

Functional headless validation only. New GPU rendering and recording remain pending recovery authorization; the original GPU halt is preserved. No new release tag.

Tested implementation `2875228d3c27e293c80087453543be0c549f3347`. Full dynamic run `20261015T234553Z_advanced_12517517-2132-4dd2-a997-3dc793ec7747` SUCCESS; same-source parked-cart control `20261015T234208Z_advanced_89cb184c-a79b-4e26-b251-29ccbbe46d2b` SUCCESS. First-goal moving interruption at the same source is intentionally FAILED, with physical stop, all component exits0, no residuals and port release PASS. All failed and superseded experiments remain available and indexed.

- `intervention.json`, `control.json`, `interrupt.json`: outcomes, actual action stages, collision/stop/cleanup results, map/TF ownership and terminal physical trajectory checks.
- `comparison.json`, `trajectory-comparison.png`/`.svg`: sampled physical travel against the published demo. Workflows, B and obstacle differ; this is not a matched planner benchmark. The orange cart is shown at its hold pose; it withdrew before passage.
- `map-coverage.json`, `map-growth.png`: actual saved SLAM grids, regional coverage and sampled structure gaps. Half-open regions use a floating-point edge epsilon of 1e-7 grid cells. The main passage survey objective excludes the parked cart bay; full east-room coverage is reported separately. Unknown occluded cells remain unknown.
- `live-path-topics.json`: readonly ROS audit, exact serialized Nav2/display Path hashes and live status/age messages. This proves data forwarding, not RViz rendering. The first audit used the wrong discovery configuration and observed no topics; its empty record is preserved in the analysis ledger, not counted as proof.
- `cpu-checks.json`: four-package build and194 passing CPU tests; one historical v0.1-only source-shape assertion excluded because advanced simulation adds scene/event support. Core physics, safety and original entry are audited independently.
- `source-audit.json`: original worktree/tag/entry, frozen upstreams, unchanged GPU guard and retained halt, current source and installed controller selection.
- `attempts.json`, `raw-index.json`, `analysis-tools.json`: all task run directories (including incomplete/preflight), exact raw-file hashes and analysis provenance. Raw runtime files remain ignored and local under `run_data/`; previous figures/packages remain under `raw/`.

Final-goal efficiency is checked independently of the Nav2 success flag: sampled path/direct-distance ratio must be <1.5 and unwrapped heading span <pi/2, rejecting terminal loops. Dynamic/control actual values: 1.1439/1.0729, 1.2064/0.6814rad. Neither run needed a terminal retry. These are two scenario demonstrations, not a reliability certification.

See `docs/advanced/optimization-review.md` for the current result and limitations. No synthetic video or offline plot is presented as a live RViz capture.

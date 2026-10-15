# Continuous mapping and visible dynamic interference — 2026-10-16

IN_PROGRESS. User requested removal of unexplained/redundant motion, visible dynamic obstacle interference and live RViz planning. Starting HEAD b191ef6af36b0a3d611604cd9de922866167ce27; both worktrees clean. Existing feature/advanced-slam-demo-v0.2 branch, /home/haoyang/humanoid_nav_ws_advanced. Tags demo-v0.1.0 and demo-v0.2.0 immutable; no push/delegation.

Diagnosis: nine separate exact-pose goals each stage 1.5m before target and stop/correct heading; east room excursion and return to A duplicate the journey. Small ADV01 box induced only a modest detour, no observable wait.

Candidate: four pass-through observation targets (0.65m milestone radius, not final arrival/safety tolerance); continue from current north-gallery pose to B with original strict arrival verification. No returning to A. Actual SLAM map saved at handover and at B; mapping continues throughout. Display actual Nav2 /plan, its age/endpoint and TF travelled trail at 5Hz; no ideal path substituted. A 3m wide cart crosses the east room, holds8s, withdraws; advanced-only bounded Wait4s recovery6 retries, unchanged safety/controller/planner parameters. Obstacle truth remains evaluation-only. Strict future acceptance: visible >=2s physical stationary period during perceived active blockage with zero final command, plus resumed travel after withdrawal and arrival; numeric detour alone is no longer sufficient. Run an optimized no-motion-obstacle control for comparison. All old failures/evidence retained.

Next: CPU checks, headless control and intervention; then staged GUI and independent recording. PIDs none before runtime. v0.1 original unchanged. Current modifications not yet validated.

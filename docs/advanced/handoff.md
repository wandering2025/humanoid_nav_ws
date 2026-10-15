# v0.2 working record

IN_PROGRESS. Started 2026-10-15T16:43:26.184106+00:00. Branch feature/advanced-slam-demo-v0.2, cwd /home/haoyang/humanoid_nav_ws_advanced. Starting HEAD 0f51d54b1dbfdde0664966118b5546109a3f6ff2, dirty none. Existing advanced worktree already matched demo-v0.1.0; reused, no reset. No agents.

Read-only review and architecture decisions recorded in architecture.md and ../evidence/advanced/start.json. No runtime processes started. GPU read-only query passed outside sandbox. Next: implement opt-in advanced SLAM/Nav2 mode and orchestrator, build with scripts/env.sh and system colcon, CPU validation first. Stable worktree untouched.

## CPU implementation checkpoint 2026-10-15T16:49:57.119512+00:00

Implementation 398d5cd2566eff48ef447437eda38aacdbe8ac00. Four packages build PASS; 134 CPU tests PASS, advanced preflight PASS. Initial interpreter/pytest setup failures retained in docs/evidence/advanced/raw/setup; no installation. Runtime/GUI/video NOT_RUN. Dirty before this record: none. PIDs: none. Next command: `./scripts/run_advanced_demo.sh --headless`, cwd `/home/haoyang/humanoid_nav_ws_advanced`; local DDS port42127, domain37. No source/safety/command changes to stable worktree.

## First physical run checkpoint

Failed run `20261015T164957Z_advanced_d0f500c3-6ed1-4bc1-a941-4e35c45c0b6e`: MAPPING_01 passed, MAPPING_02 rejected unknown route (error208). Zero contacts/falls; safe stop, all infrastructure exits0, no remaining processes, port released. Evidence headless-01.json. Corrected at 2a994a7dd4298bfbf301969994a7cc35bc211e62 with nearer observation waypoints, no planner/safety changes. Next `./scripts/run_advanced_demo.sh --headless` in advanced worktree. Dirty only this record; PIDs none.

## Full headless task / verifier checkpoint 2026-10-15T17:03:27.027048+00:00

Run `20261015T165228Z_advanced_5d6bb4c0-e5ff-4d64-98de-0d64480179b5`, tested f84154e (full SHA in evidence): all 9 mapping goals, return A, final navigation verified, error0.179m/8.17deg, contacts0/0/0, no fall, body gap1.721m sampled, safe stop, all infrastructure exits0, port released. Original FAILED retained: checker required protective wait; actual replan path deviation0.882m, actual deviation0.353m. User permits replan. Implementation bace732d48af37dc8d94432a99e4a67f79731b62 adds distinct wait/replan predicates with actual scan+costmap checks, GID TF ownership, map sidecars and independent serial window capture. CPU affected tests in /tmp/advanced-tests-03.log. Next: `./scripts/run_advanced_demo.sh --startup-check` (MuJoCo first, RViz only after readiness), then full GUI if no graphics fault. No GPU rendering yet. PIDs none; dirty only this record.

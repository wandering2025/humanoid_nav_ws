# v0.2 working record

IN_PROGRESS. Started 2026-10-15T16:43:26.184106+00:00. Branch feature/advanced-slam-demo-v0.2, cwd /home/haoyang/humanoid_nav_ws_advanced. Starting HEAD 0f51d54b1dbfdde0664966118b5546109a3f6ff2, dirty none. Existing advanced worktree already matched demo-v0.1.0; reused, no reset. No agents.

Read-only review and architecture decisions recorded in architecture.md and ../evidence/advanced/start.json. No runtime processes started. GPU read-only query passed outside sandbox. Next: implement opt-in advanced SLAM/Nav2 mode and orchestrator, build with scripts/env.sh and system colcon, CPU validation first. Stable worktree untouched.

## CPU implementation checkpoint 2026-10-15T16:49:57.119512+00:00

Implementation 398d5cd2566eff48ef447437eda38aacdbe8ac00. Four packages build PASS; 134 CPU tests PASS, advanced preflight PASS. Initial interpreter/pytest setup failures retained in docs/evidence/advanced/raw/setup; no installation. Runtime/GUI/video NOT_RUN. Dirty before this record: none. PIDs: none. Next command: `./scripts/run_advanced_demo.sh --headless`, cwd `/home/haoyang/humanoid_nav_ws_advanced`; local DDS port42127, domain37. No source/safety/command changes to stable worktree.

## First physical run checkpoint

Failed run `20261015T164957Z_advanced_d0f500c3-6ed1-4bc1-a941-4e35c45c0b6e`: MAPPING_01 passed, MAPPING_02 rejected unknown route (error208). Zero contacts/falls; safe stop, all infrastructure exits0, no remaining processes, port released. Evidence headless-01.json. Corrected at 2a994a7dd4298bfbf301969994a7cc35bc211e62 with nearer observation waypoints, no planner/safety changes. Next `./scripts/run_advanced_demo.sh --headless` in advanced worktree. Dirty only this record; PIDs none.

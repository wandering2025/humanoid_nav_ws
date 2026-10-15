# v0.2 working record

IN_PROGRESS. Started 2026-10-15T16:43:26.184106+00:00. Branch feature/advanced-slam-demo-v0.2, cwd /home/haoyang/humanoid_nav_ws_advanced. Starting HEAD 0f51d54b1dbfdde0664966118b5546109a3f6ff2, dirty none. Existing advanced worktree already matched demo-v0.1.0; reused, no reset. No agents.

Read-only review and architecture decisions recorded in architecture.md and ../evidence/advanced/start.json. No runtime processes started. GPU read-only query passed outside sandbox. Next: implement opt-in advanced SLAM/Nav2 mode and orchestrator, build with scripts/env.sh and system colcon, CPU validation first. Stable worktree untouched.

## CPU implementation checkpoint 2026-10-15T16:49:57.119512+00:00

Implementation 398d5cd2566eff48ef447437eda38aacdbe8ac00. Four packages build PASS; 134 CPU tests PASS, advanced preflight PASS. Initial interpreter/pytest setup failures retained in docs/evidence/advanced/raw/setup; no installation. Runtime/GUI/video NOT_RUN. Dirty before this record: none. PIDs: none. Next command: `./scripts/run_advanced_demo.sh --headless`, cwd `/home/haoyang/humanoid_nav_ws_advanced`; local DDS port42127, domain37. No source/safety/command changes to stable worktree.

## First physical run checkpoint

Failed run `20261015T164957Z_advanced_d0f500c3-6ed1-4bc1-a941-4e35c45c0b6e`: MAPPING_01 passed, MAPPING_02 rejected unknown route (error208). Zero contacts/falls; safe stop, all infrastructure exits0, no remaining processes, port released. Evidence headless-01.json. Corrected at 2a994a7dd4298bfbf301969994a7cc35bc211e62 with nearer observation waypoints, no planner/safety changes. Next `./scripts/run_advanced_demo.sh --headless` in advanced worktree. Dirty only this record; PIDs none.

## Full headless task / verifier checkpoint 2026-10-15T17:03:27.027048+00:00

Run `20261015T165228Z_advanced_5d6bb4c0-e5ff-4d64-98de-0d64480179b5`, tested f84154e (full SHA in evidence): all 9 mapping goals, return A, final navigation verified, error0.179m/8.17deg, contacts0/0/0, no fall, body gap1.721m sampled, safe stop, all infrastructure exits0, port released. Original FAILED retained: checker required protective wait; actual replan path deviation0.882m, actual deviation0.353m. User permits replan. Implementation bace732d48af37dc8d94432a99e4a67f79731b62 adds distinct wait/replan predicates with actual scan+costmap checks, GID TF ownership, map sidecars and independent serial window capture. CPU affected tests in /tmp/advanced-tests-03.log. Next: `./scripts/run_advanced_demo.sh --startup-check` (MuJoCo first, RViz only after readiness), then full GUI if no graphics fault. No GPU rendering yet. PIDs none; dirty only this record.

## Minimal GUI callback failure checkpoint 2026-10-15T17:05:16.985003+00:00

Run 20261015T170327Z_advanced_1d379a8f-92a7-472f-83b2-4e566587f99f FAILED before navigation: Jazzy MessageInfo is dict. Zero policy, contacts0, no fall, all exits0, no GPU monitor error/halt, no residual processes; explicit stop ACK unavailable because the callback also interrupted its spin. Evidence minimal-01.json retains failure. Fix fbb005ce84e388af233e6e64fed9947e29e57fd3: safe dict/object GID parsing; final graph exceptions cannot skip cleanup; 6 focused tests pass. No driver/settings changes. Next command `./scripts/run_advanced_demo.sh --startup-check` after port released. PIDs none, cwd advanced worktree, dirty only new evidence/this record.

## Graphics passed; TF evidence interface corrected

Minimal02 `20261015T170517Z_advanced_0c8fe097-3b4f-4c6a-9d86-b4d39f940381`: actual MuJoCo/RViz screenshots, NVIDIA renderer verified, zero commands/contact/fall, safe stop and all exits0, no graphics error. Overall FAILED solely due missing GID evidence: Jazzy rclpy omits that field (already documented by W09 source). Reuse unchanged W09 C++ RMW observer; register it in existing owned shutdown order, default v0.1 order unchanged. Implementation b216ce241083278c7c9ef961aecc9375b38087b2. Next `./scripts/run_advanced_demo.sh --headless --startup-check` verifies observer without another GPU run, then full GUI. Numerically adjusted camera8m/40deg and readable RViz phase labels from inspected screenshots. No complex host changes. PIDs none.

## Minimum gates passed 2026-10-15T17:11:36.443854+00:00

CPU authority/standing run 20261015T170848Z_advanced_33c9e8d0-ec6a-4b74-ad73-1f487e1345f2 at e4c94c65914c8184bdde6b082bd54ce3419e2f5e: SUCCESS for standing check; map→odom one actual RMW GID from slam_toolbox; scan/map/command/truth ownership pass, zero policy/contacts/fall, safe stop, all exits0 and port released. GUI rendering already inspected in minimal02, no GPU error. Next `./scripts/run_advanced_demo.sh` full GUI rehearsal with GPU monitor, no recorder. PIDs none, advanced worktree otherwise clean.

## Full GUI startup clock-step checkpoint 2026-10-15T17:15:12.841918+00:00

GUI01 `20261015T171136Z_advanced_60a91ec6-82eb-4a8a-a889-cc6e9159d366` at cdc4b5f: FAILED before goals, stale RViz heartbeat by wall mtime. Actual GPU sample timestamps show +2.422s and +2.775s wall-clock steps against monotonic elapsed time; sufficient to falsely trigger 3s guard. RViz Qt normal shutdown0, all other exits0, safe stop/contacts0/falls0/cleanup/port pass, no new host graphics fault or GPU halt. Original result retained in gui-01.json. Fix 89d48d96350bdaeb29895ceedf4e1cc687881f04 measures unchanged 3s threshold by monotonic time since a changed heartbeat file token. CPU time-jump/stall tests pass; independent recorder final hold similarly uses monotonic time. No clocks, drivers or display settings modified. Next minimal GUI standing check, no recorder; only then full GUI. PIDs none, cwd advanced worktree, dirty only this record.

## Advanced gate lifecycle checkpoint 2026-10-15T17:21:06.452754+00:00

Minimal03 graphics passed but overall FAILED: command_gate default signal teardown raised RuntimeError during take_message after acknowledged physical stop; all other components0, no GPU error. New advanced-only executable reuses unchanged SafetyGate and explicit signal flag, stops callbacks before context teardown; normal v0.1 executable unchanged. Implementation 19902995aec578e992ebedd32bbff78f5035dfeb. Correction to previous checkpoint: first heartbeat batch actually23pass/1FAIL due test import shadowing and its shell continued; fixed import now executes monotonic assertions. Broad tests subsequently had186pass/2 and187pass/1 missing-legacy-raw-fixture failures; byte-identical read-only copies of original fixtures supplied (hashes indexed), latest run188/188 PASS. Four packages build PASS. Failure logs retained. Next CPU `./scripts/run_advanced_demo.sh --headless --startup-check`, then full GUI after gate exit0. PIDs none; cwd advanced worktree, dirty only evidence/record.

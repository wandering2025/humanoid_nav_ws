# W03 verification evidence

All acceptance checks passed. `acceptance-summary.json` combines the results;
`verification-c2.json`, `verification-c3.json`, and `verification-c4.json` retain
timestamps, cwd, exact commands, exit codes, tested HEAD and implementation hashes.
The 29 implementation files have identical SHA256 values across all three phases.
The final code SHA and completion record are in the [W03 execution record](../../plans/t1_mujoco_navigation/records/W03.md).

- C1: `c1-scaffold.json`, `build.txt`; four packages built. Final rebuild is `build-final.txt`.
- C2: system/venv import comparison, generated shebangs, installed `ros2 run` and `ros2 launch` from `/tmp`.
- C3: `colcon list`, `colcon test`, `colcon test-result`, JUnit summary: 16 tests, zero errors/failures/skips.
- C4: installed modules match source, top-level launch arguments, readable parameter/path/venv errors, child-error propagation, disabled simulation, environment loading from `/tmp`, W01 baseline, copied upstream config and frozen clean repositories.

`ros2-run-initial.txt` preserves the first sandbox-restricted DDS run (process exit
0 but socket setup failed; excluded from acceptance). Subsequent local ROS runs
used approved tool escalation and completed without socket errors.
`verification-c4-failed-installed-import.json` preserves a verification-script
failure: it initially imported the pure-ROS launch module inside the simulation
venv, which has no apt `lark`. Module file lookup now uses `find_spec` without
executing launch code. ROS CLI/launch uses system Python; diagnostic uses venv.
No packages were installed or upgraded to resolve either issue.

Reproduce with the README environment followed by:

```bash
colcon build --base-paths src --event-handlers console_direct+
source "$T1_WS/scripts/env.sh"
python "$T1_WS/docs/evidence/W03/verify_w03.py"
```

The verifier runs one-shot diagnostics in ROS domain 73 and starts no daemon,
physics or viewer. ROS sockets must be permitted. Runtime logs go under the
ignored `run_data/w03_ros_logs/`; tracked evidence contains only small summaries.

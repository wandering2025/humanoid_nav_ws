# Booster T1 navigation workspace

Read `docs/plans/t1_mujoco_navigation/{总规划,执行规则,环境定位,执行看板,交接记录}.md`, the assigned step and its `records/Wxx.md` before work. Architecture contracts are in `docs/architecture/`.

Execute only the user-assigned W step, serially, without delegation. Keep the existing branch and record starting HEAD and dirty files. Preserve user edits. Use precise `git add` paths, checkpoint each verified milestone, then commit implementation/evidence before the completion/handoff records. Do not push or modify the frozen external upstream repositories.

Never replace physical walking with base-pose edits. Do not label unimplemented simulation, sensors, SLAM or navigation as working. Follow the parameter, time, TF and command ownership contracts. Build artifacts and raw runtime data stay ignored.

Use `scripts/env.sh` for the ROS/overlay/venv environment. System colcon builds the packages; simulation Python entrypoints must explicitly execute the configured venv interpreter. Do not rewrite `.bashrc`, unset `PYTHONPATH`, reinstall the environment or upgrade dependencies to mask integration errors.

Before interruption, save the step record and handoff with full SHAs, test results, dirty files, cwd, next command and remaining process PIDs. Never kill unrelated ROS/Python processes.

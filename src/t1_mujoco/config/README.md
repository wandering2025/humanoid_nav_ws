# Runtime configuration

`runtime.yaml` is the installed default parameter file. External paths are supplied with `T1_UPSTREAM`/`T1_VENV` (optional `T1_ASSETS`) or launch arguments, never a username in code. The diagnostic validates assets against the installed `booster_assets` package used by upstream.

W04 implements `t1_sim` parameters from `docs/architecture/interfaces.md` §3. `run_dir` defaults to `$T1_WS/run_data`; an empty `scene` selects the frozen upstream original. Timing is fixed at 0.02/10/0.002. All parameters are startup-only. W05: `command_map.yaml` is generated from measured data (docs/evidence/W05/build_command_map.py; ADR-010); the launch passes it as `command_map_file`, an empty value keeps `/cmd_vel` UNCALIBRATED. `warmup_sim_s=2.0` is verified by W05.

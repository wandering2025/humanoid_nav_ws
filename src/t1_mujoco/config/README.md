# Runtime configuration

`runtime.yaml` is the installed default parameter file. External paths are supplied with `T1_UPSTREAM`/`T1_VENV` (optional `T1_ASSETS`) or launch arguments, never a username in code. The diagnostic validates assets against the installed `booster_assets` package used by upstream.

W04 implements `t1_sim` parameters from `docs/architecture/interfaces.md` §3. `run_dir` defaults to `$T1_WS/run_data`; an empty `scene` selects the frozen upstream original. Timing is fixed at 0.02/10/0.002. All parameters are startup-only. W05 creates `command_map.yaml` from measurements; physical command mapping is currently unavailable. `warmup_sim_s=2.0` is a development default, pending W05 verification.

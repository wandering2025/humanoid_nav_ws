# Runtime configuration

`runtime.yaml` is the installed default parameter file. External paths are supplied with `T1_UPSTREAM`/`T1_VENV` (optional `T1_ASSETS`) or launch arguments, never a username in code. The diagnostic validates assets against the installed `booster_assets` package used by upstream.

W04 will implement `t1_sim` and extend its parameters from `docs/architecture/interfaces.md` §3. W05 creates `command_map.yaml` from measurements. Neither a simulator nor a calibrated speed map exists in W03.

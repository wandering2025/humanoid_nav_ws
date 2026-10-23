# Simulation configuration

`runtime.yaml` defines the default simulation parameters. External paths come from `T1_UPSTREAM`, `T1_VENV`, and `T1_ASSETS`. The policy runs every 20 ms over ten 2 ms physics steps.

`command_map.yaml` maps physical velocity requests to measured policy-command bands. The simulator uses a two-second startup warmup and the command gate controls admission to physical motion. All parameters are set at startup.

See the [system architecture](../../../docs/architecture.md) for sensor, command, and timing ownership.

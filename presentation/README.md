# Demonstration

The main demonstration is goal-directed exploration with a moving obstacle:

```bash
./scripts/run_exploration_demo.sh
```

MuJoCo shows physical walking and the obstacle. RViz shows the live map, latest Nav2 plan, selected target, and travelled path. Set the relative destination in [exploration_task.yaml](../src/t1_navigation/config/exploration_task.yaml).

For the guided mode, use `./scripts/run_advanced_demo.sh` and edit [goal.yaml](goal.yaml), which uses absolute map coordinates.

[Setup and options](../docs/setup.md) · [Architecture](../docs/architecture.md) · [Measured results](../docs/validation.md)

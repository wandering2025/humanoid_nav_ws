# B-point runtime interface

## W16 exploration: B relative to A

The recommended `run_exploration_demo.sh` entry reads [exploration_task.yaml](../src/t1_navigation/config/exploration_task.yaml):

```yaml
schema: 1
frame: start
goal:
  x: 11.0
  y: 0.5
```

| Setting | Rule |
|---|---|
| x / y | Finite numeric offsets in metres; +x forward at A, +y left at A |
| frame | Exactly `start`: the robot's actual base frame when standing ready at exploration start |
| Final heading | Not specified; chosen from the online approach path |
| Prior knowledge | Only B; no wall geometry, passage layout, prior map, route or observation waypoints |

Keep `schema: 1` and `frame: start`. Use numbers, not quoted strings, booleans, NaN or infinity; extra or duplicate keys are rejected. A is latched after the robot is ready, so small initial yaw drift means these offsets are not literal world/map coordinates. The explorer resolves B through odom and the current SLAM map transform. This currently relies on ideal odometry.

Save the file before launching; no rebuild is needed. Use `--task /absolute/path/task.yaml` to select a different file. Every run saves an immutable `run_data/exploration/<RUN_ID>/task.yaml`; edits during a run affect the next run. The explorer's `result.json` records the actual A/B conversion. A/B graphics and arrival checks use those same resolved coordinates.

W16 does not read `presentation/goal.yaml` or apply the legacy coordinate rectangle below. There is no offline wall/reachability check and no fixed final approach. If B is unobserved, exploration selects reachable observation goals; once an observed safe route exists, Nav2 heads to B. A target inside a wall, outside the world or still unreachable at the time limit can end in a bounded failure and safe stop. Failure does not prove that B is impossible to reach in every unknown environment.

Arrival requires position error at most 0.30m and physical settling for at least two seconds; there is no fixed final-yaw requirement. Changing B does not move walls or the dynamic obstacle. The validated default task is (11.0, 0.5); other targets may change or avoid the EXP01 encounter. See [validation](VALIDATION.md) for the actual measured runs.

## Legacy guided demo: absolute map goal

Only `run_advanced_demo.sh` reads [goal.yaml](goal.yaml). Its historical default is shown below; retain any locally edited values when using that demo:

```yaml
schema: 1
frame: map
goal:
  x: 12.0
  y: 0.0
```

| Setting | Rule |
|---|---|
| x | Finite number in **[11.0,12.0]**, inclusive, metres |
| y | Finite number in **[-0.5,0.5]**, inclusive, metres |
| frame | Exactly `map`; +X east, +Y north, A=(0,0) |
| Final heading | Fixed south: -90 degrees / -π/2 radians; no yaw setting |
| Navigation handover | Automatically `(x,y+2.5,-π/2)`; retain the original 1.5m final walking approach |

Example alternate B: `x: 11.5`, `y: -0.25`. Use numeric YAML values, not quoted strings, booleans, NaN or infinity. Unknown or duplicate keys are rejected. Keep schema/frame unchanged.

These bounds preserve the occluding partition, the broad east-room passage and cart interaction. Every point on the 2.5m vertical approach must have at least 2.4m static/parked-cart clearance. The navigation footprint remains 2.05m, stop zone 1.65m and slow zone 1.95m. The cart temporarily blocks this route; live LiDAR/costmaps handle it. This check does not provide a geometry-generated map to SLAM or Nav2.

Run `./scripts/run_advanced_demo.sh --check-goal` before presenting. An invalid or missing configuration stops startup; there is no fallback to the old B. No rebuild is needed. Broader locations or arbitrary headings require a separate route/scenario review and are intentionally rejected.

At every launch, the orchestrator rereads this exact file and writes `run_data/advanced/<RUN_ID>/presentation-goal.json`, including the original text, SHA256, B and derived handover. Navigation, the real A-to-B NavFn calculation, RViz B, MuJoCo B and arrival checks all use this snapshot. The simulator records the same goal/hash; a mismatch fails verification. Changes saved during a run take effect on the next launch. The world YAML retains its nominal metadata goal; it is not the operator interface.

Changing B does not move the robot, walls, obstacle, collision geometry, LiDAR rays or safety zones. The mapping route remains guided and fixed; the navigation route to the selected B is planned online from the newly acquired map.

See [validation](VALIDATION.md) for tested coordinates. Range validation checks geometry and workflow constraints; it is not a statistical guarantee for every coordinate or walking timing. The existing published video remains the B=(12,0) recording.

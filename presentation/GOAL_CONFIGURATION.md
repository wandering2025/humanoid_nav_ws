# B-point runtime interface

Edit only [goal.yaml](goal.yaml):

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
| Navigation handover | Automatically `(x,y+2.5,-π/2)`; retain the original1.5m final walking approach |

Example alternate B: `x: 11.5`, `y: -0.25`. Use numeric YAML values, not quoted strings, booleans, NaN or infinity. Unknown or duplicate keys are rejected. Keep schema/frame unchanged.

These bounds preserve the occluding partition, the broad east-room passage and cart interaction. Every point on the2.5m vertical approach must have at least2.4m static/parked-cart clearance. The navigation footprint remains2.05m, stop zone1.65m and slow zone1.95m. The cart temporarily blocks this route; live LiDAR/costmaps handle it. This check does not provide a geometry-generated map to SLAM or Nav2.

Run `./scripts/run_advanced_demo.sh --check-goal` before presenting. An invalid or missing configuration stops startup; there is no fallback to the old B. No rebuild is needed. Broader locations or arbitrary headings require a separate route/scenario review and are intentionally rejected.

At every launch, the orchestrator rereads this exact file and writes `run_data/advanced/<RUN_ID>/presentation-goal.json`, including the original text, SHA256, B and derived handover. Navigation, the real A-to-B NavFn calculation, RViz B, MuJoCo B and arrival checks all use this snapshot. The simulator records the same goal/hash; a mismatch fails verification. Changes saved during a run take effect on the next launch. The world YAML retains its nominal metadata goal; it is not the operator interface.

Changing B does not move the robot, walls, obstacle, collision geometry, LiDAR rays or safety zones. The mapping route remains guided and fixed; the navigation route to the selected B is planned online from the newly acquired map.

See [validation](VALIDATION.md) for tested coordinates. Range validation checks geometry and workflow constraints; it is not a statistical guarantee for every coordinate or walking timing. The existing published video remains the B=(12,0) recording.

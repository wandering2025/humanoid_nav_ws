# Simulation worlds

Coordinates are world metres: +x forward, +y left, +z up; yaw is counterclockwise in radians. Box `size` is full extent. Bounds describe the inner wall faces.

| Layout | Purpose |
| --- | --- |
| `advanced_world` | Occluded A-to-B route with a north passage and dynamic-obstacle slot |
| `exploration_south` | Alternate layout requiring a south-side exploration route |
| `static_world` | Saved-map navigation around a central island |
| `dynamic_crossing_world` | Crossing, blocking, and sudden-approach scenarios |
| `calibration_world` | Walking and command-map measurements |
| `stress_test_world` | Geometric clearance experiments |

Layouts define collidable geometry used by physics and LiDAR. Scene events are configured separately in `t1_bringup/config/`. The exploration process receives live sensor-derived maps; world layouts and obstacle event trajectories belong to the simulator.

[Architecture](../../../docs/architecture.md) · [External model notices](../../../THIRD_PARTY_NOTICES.md)

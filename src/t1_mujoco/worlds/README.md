# W07 flat navigation scenes

All coordinates are world metres, +X forward, +Y left, +Z up, yaw radians CCW.
`size` is full box extent, not MuJoCo half-size. Bounds describe inner wall faces.
A=(0,0), yaw=0; B=(6,0), yaw=0. A/B are endpoints only; no route is supplied to navigation.

`calibration_world` has inner walls x=-2,8 and y=-5,5. `static_world` adds a central island requiring a north/south choice, a corner and a fixed box. `dynamic_crossing_world` leaves the main corridor clear and reserves crossing/approach regions. `stress_test_world` has a 1.9 m central gate followed by a two-sided island and three activity regions. Activity bounds and obstacle slots are test metadata, never planner inputs or future trajectories. Slots specify dimensions for W12; W07 instantiates no moving objects.

Walls/boxes are 1.5 m high, visible and collidable, group 2 (environment ray candidates for W08). Ground and the complete robot retain original physics, collision settings and visual groups. Small/overhanging obstacles are outside this planar test design. A 10x10 m floor region keeps candidate A/B paths under the W05 17 m budget; actual planned path limits remain for W11.

The 1.9 m gate is a geometry constraint, not a certified safety clearance. C3 measures a conservative body polygon from W05 trajectory replays; W10 must add stopping/latency margins before navigation. Layouts are original project contributions. Upstream provenance and exact modification boundary: [sources.md](../../../docs/evidence/W07/sources.md).

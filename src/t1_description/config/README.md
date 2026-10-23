# Robot description and visualization

The launch wrapper resolves the external Booster T1 URDF and mesh paths. Joint states drive the 23 moving joints; robot_state_publisher uses simulation time. The state bridge owns `odom → base_footprint → base_link`.

`footprint.yaml` contains the conservative measured body footprint. `scene.rviz` and `state.rviz` provide robot, TF, and sensor displays. Display materials and camera settings affect visualization only; physical contact geometry stays in the simulator.

[System architecture](../../../docs/architecture.md)

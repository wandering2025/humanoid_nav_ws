# Mapping and localization resources

W09: `slam.yaml` follows installed Jazzy SLAM Toolbox online_async defaults with
project frames, W08 range and mapping cadence. `localization.yaml` follows installed
Jazzy Nav2 AMCL/map_server parameters for the saved LiDAR map. `mapping.rviz` displays
map/scan/robot/TF and exposes the one-shot 2D Pose Estimate tool.

Use t1_bringup mapping.launch.py or localization.launch.py; each composes exactly
one map→odom owner. Map provenance is in ../maps/static_world.meta.yaml.
See docs/architecture/mapping_localization.md and docs/evidence/W09/README.md.

W10 velocity_smoother/collision_monitor and W11 autonomous Nav2 are still pending.

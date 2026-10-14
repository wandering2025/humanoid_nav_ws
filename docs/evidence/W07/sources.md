# Sources and contribution boundary

- Official assets: https://github.com/BoosterRobotics/booster_assets, frozen commit `12a97516f57469078707afb587a5f84c1cb110d1`; local license `/home/haoyang/robot_sim_sources/booster_assets/LICENSE`, BSD-3-Clause. Notice retained in `booster-assets-LICENSE.txt`.
- Controller/policy: https://github.com/BoosterRobotics/booster_deploy, frozen commit `7bb1462e2ac742eba5c01659b815e2e99fbe617f`; license recorded in W01. No controller/policy content copied or altered in W07.
- Referenced external files: `robots/T1/T1_23dof.xml`, `T1_23dof.urdf`, their `meshes/*.STL`. No robot mesh or weights redistributed here.
- Reused: original ground plane, checker material/sky texture, two lights, full robot asset/default/body/joint/actuator definitions and physics attributes. MJCF generated in ignored `run_data/cache/scenes/`; meshdir resolved to frozen assets. Ground contact parameters are unchanged.
- Original project work: four YAML layouts, boundary walls, obstacle dimensions/positions, A/B endpoints, activity regions/slot schema, assembly/provenance checks, model consistency validation, conservative footprint, top-view illustrations and launch integration.
- Generated model changes: descriptive names for originally unnamed geoms/lights; absolute mesh directory; append named group-2 collidable box geoms in worldbody. No duplicate asset/default/worldbody merge, no added joints/actuators, no fixed free base, no collision removal.
- Robot description references the matching 23DOF official URDF through the W06 wrapper, resolves external mesh URIs, adds only identity `base_link→trunk`. No `base_footprint→base_link` static TF; W06 owns its dynamic pose. W08 owns future LiDAR mounting.

Each generation records source XML/config/mesh/license SHA256 and assets Git SHA. Source Git SHA must match the frozen value and its working tree must be clean. Local checkout remains unchanged.

W07 ROS visualization makes the original translucent CAD visual materials opaque and uses scene.rviz with a brighter background and camera following base_footprint. This changes only visual material alpha/camera, not link/mesh transforms or physical contact properties. state.rviz remains available.

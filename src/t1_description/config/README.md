# T1 visualization and footprint

The W06 launch wrapper in t1_bringup/description.py references the frozen BSD-3-Clause external T1_23dof.urdf and resolves mesh URIs. robot_state_publisher uses simulation time, publish_frequency=100.0; 23 JointStates at 50 Hz drive all moving frames. base_link→trunk is identity; W06 owns dynamic odom→base_footprint→base_link, so no duplicate static base TF is provided. LiDAR mounting belongs to W08.

W07 independently verifies zero/random configurations, all body FK on real gait snapshots, and raw STL visual bounds against compiled MuJoCo mesh vertices. The official model needs no coordinate corrections. `footprint.yaml` is a conservative rectangle in base_footprint, includes collision and visual geometry (head, hands, torso, feet), measured from W05 representative physical trajectory replays with 0.05 m extra per side and outward centimetre rounding. It is future navigation input; W07 launches no Nav2. Stopping/sensor/latency margins remain for W10.

See docs/evidence/W07/description-envelope.json, geometry.md and sources.md for measurements and limitations. Foot contact bodies are left_ankle_roll_link and right_ankle_roll_link; all original collision settings are retained.

W07 ROS visualization makes the original translucent CAD visual materials opaque and uses scene.rviz with a brighter background and camera following base_footprint. This changes only visual material alpha/camera, not link/mesh transforms or physical contact properties. state.rviz remains available.

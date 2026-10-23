# Third-party notices

The project integrates the following components. External repositories, robot meshes, and policy weights are retrieved separately; they are not bundled in the public source snapshot.

| Component | Role | Source / license |
| --- | --- | --- |
| Booster Deploy | T1 pretrained walking policy and deployment utilities | [BoosterRobotics/booster_deploy](https://github.com/BoosterRobotics/booster_deploy), Apache-2.0 at the pinned revision |
| Booster Assets | Robot MJCF, URDF, meshes, and model resources | [BoosterRobotics/booster_assets](https://github.com/BoosterRobotics/booster_assets), BSD-3-Clause at the pinned revision |
| MuJoCo | Rigid-body dynamics and ray casting | [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco), Apache-2.0 |
| ROS 2 / Nav2 | Middleware, planning, control, costmaps, and collision monitoring | [ros2](https://github.com/ros2), [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2); retain each installed package's notices |
| SLAM Toolbox | Online occupancy mapping and map-frame correction | [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox); retain its distribution license and notices |

Exact Booster revisions are in [dependencies.repos](dependencies.repos). The repository's [Apache-2.0 license](LICENSE) applies to this project's original code and documentation. Upstream assets and dependencies retain their own terms.

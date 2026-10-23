# Humanoid Navigation with Booster T1

**Give the robot a destination. Let it discover the route and walk there.**

A ROS 2 and MuJoCo project connecting physical bipedal locomotion, LiDAR SLAM, goal-directed exploration, and Nav2. Starting with a relative goal, the T1 builds a map, selects observation targets, finds a route through observed free space, and responds to moving obstacles along the way.

<table>
  <tr>
    <td><img src="docs/media/exploration-mujoco.png" alt="Booster T1 facing a moving orange obstacle in MuJoCo" width="480" /></td>
    <td><img src="docs/media/exploration-rviz.png" alt="RViz showing the live LiDAR map, Nav2 plan, travelled path and collision-monitor zones" width="480" /></td>
  </tr>
  <tr>
    <td align="center">Physical walking and obstacle interaction</td>
    <td align="center">Live mapping, planning, and execution</td>
  </tr>
</table>

*Actual screenshots from the recorded autonomous exploration run.*

## Core capabilities

- **Navigation grounded in a walking robot.** The official T1 policy drives joint targets and PD torques at 500 Hz physics / 50 Hz policy rates. A calibrated gait adapter connects Nav2 requests to the measured walking and turning envelope.
- **A destination is enough.** The explorer scores reachable frontiers by travel cost, progress toward the goal, information gain, and visit history. It switches to the destination as soon as an observed route becomes available.
- **Separate perception paths for mapping and obstacle response.** An observation-history filter removes transient returns from the SLAM input. Nav2 and Collision Monitor retain the unfiltered scan, keeping moving obstacles visible to the command chain.
- **Results tied to physical execution.** Each run records decisions, map snapshots, trajectories, contacts, arrival error, and process exits. An evaluator checks these independently of the navigation controller.

The project focuses on the engineering boundary between a learned walking policy and a classical navigation stack: turning perception and plans into executable humanoid motion.

## Demonstrated behavior

In the showcased run, the robot received **B = (+11.0, +0.5) m relative to its starting frame**. It selected six exploration targets, discovered the passage around an occluding wall, waited for a moving obstacle, and continued to B.

| Measurement | Recorded result |
| --- | ---: |
| Selected paths checked against their observed costmaps | 7 / 7 |
| Stationary wait during the obstacle encounter | 17.72 s |
| Travel after the obstacle withdrew | 4.13 m |
| Maximum goal error during the final two seconds | 0.095 m |
| Environment / obstacle contacts; falls | 0; 0 |

![Online SLAM map with the measured route and selected exploration targets](docs/media/exploration-map.png)

[Run evidence and validation](docs/validation.md) includes source revisions, the static control, alternate layouts, and load-test results. The simulation uses ideal odometry and attitude compensation; world geometry and obstacle trajectories stay outside the exploration and planning interfaces.

## Quick start

Use **Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, and MuJoCo 3.15.0**. Follow the [one-time setup](docs/setup.md) to install the pinned Booster dependencies and build the four ROS packages.

From the repository root, with `T1_VENV`, `T1_UPSTREAM`, and `T1_ASSETS` configured:

```bash
# Check the environment and calibrated runtime files.
./scripts/run_exploration_demo.sh --preflight

# Run the complete mission without windows.
./scripts/run_exploration_demo.sh --headless

# Open MuJoCo + RViz on the documented WSL2 / NVIDIA display profile.
./scripts/run_exploration_demo.sh
```

Set the destination in [exploration_task.yaml](src/t1_navigation/config/exploration_task.yaml):

```yaml
schema: 1
frame: start
goal: {x: 11.0, y: 0.5}  # metres: forward, left
```

Changes apply on the next run. Add `--static` for the obstacle-free control or `--hold` to keep the final view open. **Ctrl+C requests a latched stop and cleans up the run's processes.** Results are written to `run_data/exploration/<run-id>/final-result.json`.

## System architecture

![Five-layer architecture showing simulation, mapping, exploration, navigation, command supervision, and the closed control loop](docs/media/system-architecture.png)

The blue path keeps unfiltered scans available to costmaps and Collision Monitor. The green loop returns the final command to the walking policy. [View the vector diagram](docs/media/system-architecture.svg).

| Package | Responsibility |
| --- | --- |
| `t1_mujoco` | Physics, policy execution, calibration, LiDAR, and ROS state |
| `t1_description` | Robot model integration, footprint, and visualization |
| `t1_navigation` | SLAM, localization, planning, controller, and safety configuration |
| `t1_bringup` | Launch composition, exploration algorithms, and command ownership |

[Architecture](docs/architecture.md) · [Setup and commands](docs/setup.md) · [Validation](docs/validation.md) · [Development](CONTRIBUTING.md)

## Credits and license

Project integration includes the simulator/ROS bridge, gait calibration, compensated LiDAR, exploration manager, scan filter, command supervision, and evaluation tools. Robot models and the pretrained walking policy come from **Booster Robotics**; mapping and navigation use **SLAM Toolbox** and **Nav2**.

Project code is licensed under [Apache 2.0](LICENSE). External models, policies, and libraries retain their respective licenses; see [third-party notices](THIRD_PARTY_NOTICES.md).

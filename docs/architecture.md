# System architecture

The system closes the loop from LiDAR observations to map-based decisions, Nav2 commands, joint torques, and new observations. Its central integration task is to make a mobile-robot navigation stack execute through a bipedal walking policy.

## Engineering decisions

| Engineering requirement | Design decision | Result / validation |
| --- | --- | --- |
| Execute velocity requests through a gait with measured speed and turn bands | Calibrate physical velocity separately from policy input; adapt commands before final collision filtering | Nav2 drives physical walking through the established command map |
| Build a planar map while the torso rolls and pitches | Cast a torso-mounted multilayer LiDAR and compensate attitude before planar projection | A stable scan feeds SLAM, costmaps, and obstacle response during walking |
| Reach a destination beyond initially visible free space | Rank reachable frontiers, validate paths with NavFn, and preempt exploration when B becomes reachable | Six observation targets precede the final approach in the showcased run |
| Keep moving obstacles out of the structural map while reacting immediately | Filter only the SLAM branch; keep the unfiltered stabilized scan for Nav2 and Collision Monitor | Recorded wait/resume response; the final map clears the former blockage |
| Preserve final command authority across ROS nodes | One command gate, a smoother, Collision Monitor, and a bridge that only reduces commands | Latched stops, publisher ownership checks, and physical settling checks |

## Perception and exploration

MuJoCo runs at **500 Hz** and the Booster walking policy at **50 Hz**. The LiDAR uses **9 vertical layers × 360 horizontal directions**, **10 Hz**, and a **12 m** range. The simulator publishes ideal planar odometry, TF, joint states, and simulation time from a consistent physical snapshot.

The mapping filter compares returns with previously observed free cells and stable surfaces. Transient returns become invalid samples on `/scan/mapping`; each scan retains its timestamp and geometry. SLAM Toolbox continuously publishes `/map` and owns `map → odom`. The original attitude-compensated `/scan` feeds both Nav2 costmaps and Collision Monitor.

The exploration process receives a relative B, `/map`, the live global costmap, TF, simulation time, and safety state. It uses Nav2's `ComputePathToPose` and `NavigateToPose` actions to:

1. Test for a route to B through observed, reachable cells.
2. Otherwise score reachable frontier viewpoints by travel cost, goal distance, information gain, and revisit penalty.
3. Validate selected paths cell by cell, execute them, and reassess as observations change.
4. Switch to B once its route is established, then verify arrival and stationary settling.

The start frame A is latched when exploration begins: +x is forward and +y is left. B is transformed through that frame and the current SLAM correction. The simulator alone reads scene geometry and obstacle event trajectories. The evaluator reads ground truth for measurements; those topics do not supply exploration targets or planner inputs. Ideal odometry and truth-based attitude compensation are explicit simulation inputs.

## Navigation and command ownership

Exploration uses **NavFn**, **Rotation Shim**, and **Regulated Pure Pursuit**. Its behavior tree replans every five seconds or when the path becomes invalid. The gait adapter maps requests into calibrated walking and turning bands before the final protection layers.

```text
Nav2 → gait adapter → command gate → velocity smoother
     → Collision Monitor → /cmd_vel → reducing-only bridge → walking policy
```

Collision Monitor uses a **1.65 m stop zone** and **1.95 m slow zone**. A stale sensor, invalid command, lost source heartbeat, pause, or explicit stop revokes the motion permit. Recovery requires healthy inputs and explicit arming. The final bridge cannot increase a command reduced by Collision Monitor.

## Run ownership and evaluation

The launcher owns a local DDS discovery server and every child process in its run. It records process identities, requests a physical stop before shutdown, waits for its children, and checks port recovery. The exploration finalizer retains nonzero component exits in the mission verdict.

Run artifacts include the task snapshot, parameter hashes, decision events, observed maps, command/pose traces, contact counts, and final result. These make the route choice and physical outcome inspectable together. [Validation](validation.md) reports the demonstrated scenarios and operating conditions.

The separate guided mode uses configured mapping viewpoints and an absolute map goal. During its obstacle-navigation phase it holds the completed SLAM correction while live LiDAR continues to update costmaps. Use the exploration entrypoint for observation-driven target selection and continuously active SLAM.

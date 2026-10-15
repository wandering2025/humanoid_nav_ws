# W11 autonomous static navigation (ADR-015, ADR-016)

Scope: Nav2 Jazzy 1.3.13 on the saved static_world v2 map, flat ground, frozen W05 gait, W08 compensated scan, ideal truth odometry, AMCL localization, W10 safety chain. Static obstacles only; no dynamic prediction. This is a simulation prototype, not a hardware or safety certification.

## Graph

```text
t1_sim ──/scan,/odom,/tf,/clock──► map_server+AMCL (map→odom)
                                  ► planner_server(NavFn) ─/plan─► bt_navigator ─► controller_server
                                                                                     RotationShim→MPPI DiffDrive
controller_server, behavior_server(Wait only) ─/t1/nav/cmd_vel_raw─► t1_navigation_adapter
t1_navigation_adapter ─/t1/cmd_navigation─► t1_command_gate ─/cmd_vel_nav─► velocity_smoother
  ─/cmd_vel_smoothed─► collision_monitor ─/cmd_vel─► t1_sim bridge (only reduces) ─► W05 map ─► policy
```

`navigation.launch.py` (mode `navigation`) starts the W09 localization pair and the W10 chain with `command_source:=navigation`; their lifecycle managers start `lifecycle_start_delay` (3 s) after the managed nodes. After `nav_start_delay` (8 s wall) it creates the Nav2 group, the adapter, the readiness starter, RViz (if requested) and the optional one-shot `initial_pose`. Under W11R1 (ADR-019), localization/navigation use the bounded lifecycle manager; the W10 safety manager is unchanged. The starter checks actual lifecycle responses, AMCL initialization acknowledgement, READY, advancing clock, fresh scan and map→odom→base_footprint, then confirms real action/service RPCs. The creation delays only pace discovery load. All consumers use sim time, and every command message is a `geometry_msgs/Twist`.

## Frozen parameters

- `src/t1_navigation/config/navigation.yaml`:
  - W11R1 ADR-023: Smac differential lattice, forward curves (1.0 m radius) and in-place orientation, reverse expansion disabled, no unknown space; replaces NavFn position search.
  - Costmaps: robot circle of radius 2.05 m with inflation to 2.6 m. Local costmap 10×10 m on `/scan` (`inf_is_valid`, raytrace 6.5 m).
  - RotationShim: in-place 0.65 rad/s, threshold 0.6 rad, `rotate_to_goal_heading`.
  - MPPI DiffDrive: vx [0, 0.45], \|wz\| ≤ 0.45, 56×0.05 s horizon.
  - StoppedGoalChecker: 0.20 m / 0.20 rad.
  - PoseProgressChecker: 0.3 m / 0.5 rad / 20 s.
- BT `behavior_trees/navigate_to_pose_w11.xml`: replans on a new goal, an invalid path or a 5 s path age; recovery is clear-costmap + Wait (3 retries). No spin and no backup.
- Adapter shaping (ADR-016):
  - vx ≥ 0.315 passes unchanged.
  - A sub-gait request that curves tighter than 0.70 m becomes an in-place turn at 0.6–0.85 rad/s.
  - Any other forward intent ≥ 0.01 m/s becomes 0.33 m/s; everything else becomes zero.
  - It sits before the smoother and monitor, so the bridge's only-reduce rule and the Collision Monitor zones still apply.
- Goal tool `ros2 run t1_bringup navigate_to_pose X Y YAW`:
  - Explicitly arms the gate, then enforces one 120 s simulation-time limit for the whole task.
  - By default send the final position and heading directly to the constrained planner. Explicit legacy staging remains available through `--approach-distance`.
  - Complete and settle any requested staging action before the terminal goal; both require localized position/yaw verification. At most 3 corrections are shared across the whole task.
- Cancel calls `/t1/safety/stop` through the adapter; a later goal needs a new arm. Preemptive replacement does not stop.

## Why the bipedal-specific parts exist

The T1 policy cannot creep: below 0.315 m/s it stands. In-place turns only work at 0.56–0.85 rad/s. After an in-place turn stops, the stance rebounds 0.1–0.4 rad against the turn direction, and the gait's yaw oscillation fools Nav2's instantaneous "stopped" test. W11 C2 measured each of these effects (`docs/evidence/W11/c2-dev-experiments.json`); the design choices that answer them are:

- Walking speeds are lifted.
- In-place turns are only commanded inside the calibrated band.
- The W11R1 global path includes forward curvature and the final heading.
- Arrival is verified on the localized pose over a stationary window.

The original corridor checks in `c2-cases-feasibility.json` remain historical evidence; forward-curvature reachability and physical tracking require the W11R1 independent matrix.

## Limits

- **Safety zones**: the 2.05 m planning circle and W10's omnidirectional 1.65/1.95 m zones restrict where the robot can go. Passages need ≥4.1 m of width and goals ≥2.05 m (2.25 m registered) of clearance. Inside the slowdown circle, requests ≤0.63 m/s stop the robot (W10 design).
- **Scan coverage**: the scan is a planar projection of 9 instantaneous beams. Objects outside 0.15–1.20 m height or between beams can be missed (W08).
- **Yaw rebound**: a terminal yaw rebound can still exceed tolerance after 3 corrections, which reports FAILED with an explicit exhaustion reason. Measured in-place-only arrivals ranged 7–28°.
- **Localization assumptions**: localization and stationarity use AMCL on ideal odometry. Truth is read only by the W11 evaluator (`docs/evidence/W11/nav_run.py`).

## W12 sensor-driven dynamic obstacles

The `dynamic_crossing_world` scene has its own SLAM map. Mapping builds no mocap
bodies (`nmocap=0`); navigation opts into exactly one collidable group-2 mocap box
with `events_file` and `scenario`. Robot dimensions stay nq=30/nv=29/nu=23.
`SceneEvents.update` writes only mocap position/quaternion before each 2 ms
physics substep on the owner thread. Pause freezes motion; episode reset restores
the staged position and clears the trigger. Kinematic obstacles cannot be pushed.

`src/t1_bringup/config/w12_events.yaml` defines crossing, prolonged blocking and
sudden approach, including dimensions via scene slots, velocities, trigger,
seed, permitted response and termination. Timing is independent of scene bytes;
its absolute path, SHA256, scenario and seed enter each run manifest. The W09
map/scene byte-hash contract remains unchanged. Editing scene YAML requires a
fresh SLAM map and localization checks; editing events does not change the map.

The event source uses robot motion/position only to trigger an experiment. Nav2
gets `/scan` and the resulting costmaps. `/t1/eval/obstacle` (String JSON, 50 Hz,
Reliable/Volatile depth 10) exposes the current entity position, size, phase,
episode and simulation timestamp to the evaluator only. It publishes no future
trajectory and no TF. Tests audit that no navigation, localization or costmap
node subscribes. This is reactive avoidance without target tracking/prediction.

W10 zones, smoothing and the ADR-016 adapter/goal tool/launch delays remain.
The slowdown zone stops this gait at navigation speeds. Crossing and approach
can therefore wait and automatically resume after scans clear; prolonged blocking
must stay present while the robot actually detours. The approach configuration
allows braking travel and obstacle advance, then withdraws so B's 1.5 m approach
corridor is free. The stop zones do not control obstacle motion or promise that
a kinematic obstacle will never enter them; measured gaps and contact events are
reported alongside the outcome.

W12 mapping uses a separate runtime file with 1 cm W08 range noise (seed 0).
Noiseless parallel wall endpoints exposed Karto's boundary rounding issue; the
failed map is retained. The new SLAM map meets the original W09 quality limits.
Dynamic-world localization uses `w12_localization.yaml` (sigma_hit 0.05 m, all
other parameters as W11). The W11 map, scene, safety and navigation configuration
are preserved. Mapping and three independent AMCL initializations precede dynamic
experiments. See [W12 evidence](../evidence/W12/README.md).

## W11R1 readiness and terminal contract (ADR-019)

The navigation-mode `bounded_lifecycle_manager` owns only localization/navigation
transitions and bonds. Each service discovery/response is bounded to 2 wall seconds,
with at most three state-reconciled attempts. A lost change-state response is
followed by GetState; an already applied transition is not replayed. Bonds retain
4 s timeout / 0.1 s heartbeat. Failure requests W10 stop, deactivates reachable
managed nodes and requires a fresh launch; no automatic re-arm/restart. The W10
manager and safety files remain unchanged. Starter failure shuts down its launch.

Graph visibility and active state are insufficient. Before arming, starter and
navigate_to_pose test FollowPath with an empty path: accepted ACK followed by the
expected ABORTED/INVALID_PATH=103 proves the actual response channel without any
motion. The adapter admits no commands without a current NavigateToPose epoch.
Both clear-costmap services must actually respond. The expected probe aborts are
startup diagnostics, not task failures. NavigateToPose acceptance is checked on
the actual goal with a bounded ACK wait. The BT request budget is 500 ms (C09
measured about 270 ms); motion watchdogs and 120 s task budget are unchanged.

The adapter clears cached Twist at every NavigateToPose/FollowPath epoch change,
requires a current FollowPath accepted no earlier than the current navigation goal,
and blocks canceled/aborted epochs while requesting the latched W10 stop. This
also blocks a late orphan FollowPath after its parent BT aborts. No publisher is
added to the command chain. External preemption still works; staged approach no
longer preempts at the old action's completion boundary.

The optional legacy 1.5/1.125/0.75 m costmap-selected staging positions remain. Each staging
action must complete and settle: 0.5 s velocity window below .05 linear/angular,
continuously for 2 s, position/yaw within the existing .20 m/.20 rad internal
bounds. Only then may the final goal be sent. Both stages share three corrections:
position errors re-send the nominal pose; yaw-only errors integrate localized
residual with aim offset capped at .5 rad. No nominal tolerance is relaxed. Action
ABORTED (including error code 0), nonstationarity, exhausted corrections, clock
stall, timeout or terminal forward overshoot >.30 m cause explicit failure,
stop and cancellation. Overshoot is not recovered by an unvalidated reverse move.

This candidate still requires the independent verification matrix; the former
W11/W12 samples and W13 formal-v1 failures retain their historical meanings.

W11R1 H9 candidate uses the installed Smac differential lattice planner with
1.0 m forward curves and the already supported in-place orientation change.
Reverse expansion is disabled. The primitive file hash is frozen in ADR-023.
Goal heading is part of the search state. Default navigation sends the final
pose directly; explicit legacy staging must still pass joint verification.
The 2.05 m collision circle, unknown-cell policy, zero planner tolerance and
MPPI/RotationShim safety chain are unchanged. Failed H6/H7/H8 experiments are
preserved and superseded; pure Dubin could not turn away after a late blocker.

ADR-024 candidate lowers only the W11 raw yaw-intent threshold from .20 to
.05 rad/s: observed active path corrections .058–.15 were previously discarded.
Actual in-place output remains .60 rad/s (clamped to frozen [.56,.85]); W05
dead zones, walking limits and W10 are unchanged. The goal epoch gate still
blocks all commands without a current active navigation/controller goal.

W11R1 ADR-025：`t1_navigation::ExactGoalLattice`复用已安装SmacPlannerLattice并恢复最后path pose的精确请求orientation。搜索内部角度离散不应改变对外连续终点语义，尤其是±0.2rad附近的回弹纠偏。所有平移样本与圆形2.05m碰撞检查原样保留；真实规划服务测试覆盖+.238034/−.216578请求。

W11R1 ADR-026：MPPI PathAlignCritic使用可行格点路径的朝向；旧二维路径的XY-only成本会把同位置的侧向轨迹看作完全对齐。其余成本权重/激活距离不变，实际Nav2 critic行为测试验证区别，而非只断言YAML值。

W11R1 ADR-027（替代ADR-026候选控制配置）：局部控制采用已安装RegulatedPurePursuit，直接请求.33–.40m/s前进与允许的原地对准，避免MPPI连续亚步态预测与实际整形后的不一致；全局ExactGoalLattice、安全链、终点核验/预算保持。MPPI/RotationShim配置不再启用，旧实验记录不删。

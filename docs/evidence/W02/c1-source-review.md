# W02 C1 上游源码审查摘要

审查时间：2026-10-13T19:44–19:50 UTC。被审查版本：booster_deploy `7bb1462e2ac742eba5c01659b815e2e99fbe617f`、booster_assets `12a97516f57469078707afb587a5f84c1cb110d1`，两仓审查前后 `git status --short` 均为空。
完整读取：`scripts/deploy.py`、`booster_deploy/controllers/{mujoco_controller,base_controller,controller_cfg}.py`、`booster_deploy/robots/t1.py`、`tasks/locomotion/{__init__,locomotion}.py`、`tasks/locomotion/robots/t1/__init__.py`、`booster_deploy/utils/registry.py`、`utils/policy_runner.py`（TorchScript 部分）、`utils/isaaclab/math.py::quat_apply_inverse`、`T1_23dof.xml` 头部/执行器/传感器段。机读探针结果：`c1-probe.json`（15/15 PASS）。

## 1. 调用链与方法契约

`deploy.py --task t1_walk --mujoco` → `get_task("t1_walk")` 返回 `T1WalkControllerCfg1`（t1/__init__.py:73–80）→ `MujocoController(cfg).run()`。

| 方法 | 位置 | 行为与对历史/动作/关节顺序的要求 |
| --- | --- | --- |
| `BaseController.__init__` | base_controller.py:193 | 建 `BoosterRobot`、`VelocityCommand`（全 0）、`cfg.policy.constructor(cfg.policy, self)` 即 `LocomotionPolicy`，加载 TorchScript |
| `MujocoController.__init__` | mujoco_controller.py:15–50 | `MjModel.from_xml_path(cfg.robot.mjcf_path)`（**模型路径是唯一注入点**）；覆盖 `opt.timestep=physics_dt`；`qpos = concat(init_pos, init_quat, default_joint_pos)` **要求 nq==30**；额外建 ghost MjData |
| `start()` | base_controller.py:204 / mujoco_controller.py:52 | 步数清零、`is_running=True`、`policy.reset()`：`obs_history=None`、`last_action=0`、`filtered_dof_target=default` |
| `update_state()` | mujoco_controller.py:161–184 | `qpos[7:]`→joint_pos，`qvel[6:]`→joint_vel，`qfrc_actuator[6:]`→torque，`qpos[:3]/[3:7]`→root pos/quat，`qvel[:3]`→`root_lin_vel_b`（**名称错误，实为世界系**，见 §4），`qvel[3:6]`→`root_ang_vel_b`（确为机体系） |
| `policy_step()` | base_controller.py:212 | `_step_count+=1`；调用 `inference()` |
| `inference()` | locomotion.py:156–182 | 观测 72 维/帧：`ang_vel_b(3) proj_gravity(3) cmd(3) dq(21) dqd(21) last_action(21)`；历史 10 帧，首帧 9 个零帧+当前帧（`history_init="zeros"`），之后 roll；输出 21 维，clip ±100；`last_action` 回写；映射回 23 维真实顺序，`target = default + 0.25*action`；头部两关节不在策略中，目标恒为默认值 |
| `compute_observation()` | locomotion.py:113–154 | 命令**原值**直接进入观测（`vel_command.lin_vel_x/lin_vel_y/ang_vel_yaw`，无归一化）；`projected_gravity_z > -0.5`（倾角 >60°）时调用 `controller.stop()`，该步动作仍返回 |
| `ctrl_step()` | mujoco_controller.py:221–246 | **先** `update_vel_command()` 读 stdin（新命令下一 policy 步才进观测，晚一拍），再 10 次：`ctrl = clip(kp*(target-q) - kd*qd, ±effort_limit)` 全数组赋值 + `mj_step` |
| `update_vel_command()` | mujoco_controller.py:138–159 | stdin 三数；只对 vx 做 `clamp_vx`（±1.0），**vy/yaw 未限幅**；真机路径（booster_robot_controller.py:723–725）为归一化×max |
| `run()` | mujoco_controller.py:248–280 | `launch_passive` viewer；循环 `sleep(0.02)`→update_state→policy_step→ctrl_step→viewer.sync；暂停时不推进物理；退出条件 viewer 关闭或 `is_running=False` |
| `stop()` | base_controller.py:226 | 仅 `is_running=False` |

## 2. 时步一致性

`ControllerCfg.policy_dt=0.02`，`MujocoControllerCfg.decimation=10`，`__post_init__` 设 `physics_dt=policy_dt/decimation=0.002`；MJCF 原值也是 0.002，但运行时以配置覆盖为准。探针 PASS。运行器选择：**策略周期 0.02 s（50 Hz）、物理 0.002 s、每策略步 10 个物理步**，不允许本项目自行改动（会改变策略训练分布）。`_elapsed_s` 用步数×policy_dt 计算，非 `data.time`；本项目时间真相只用 `data.time`。

## 3. 关节/执行器顺序（探针核实）

- joint0 为 free（`world_joint`，body `trunk`，qposadr 0，dofadr 0）；23 个 hinge 的 qpos 顺序 == `RobotCfg.joint_names`，qposadr 7..29、dofadr 6..28 连续。
- 23 个 motor 的目标关节顺序 == `RobotCfg.joint_names`，gear 全 1，`ctrllimited` 未设；力矩限幅来自 `effort_limit`（与 MJCF forcerange 数值一致）。
- `sim_joint_names`（IsaacLab 顺序）只用于 `RobotData.real2sim/sim2real` 索引；LocomotionPolicy 使用自身 `policy_joint_names`（21 个，缺 `aahead_yaw_joint`/`aahead_pitch_joint`）。
- URDF `T1_23dof.urdf` 根 link=`trunk`，23 个可动关节同名；另有固定 `head_realsense_rgb_joint`。`T1_locomotion.urdf` 只有 12 个可动关节，**不得**用于 /joint_states 发布对应。

## 4. 坐标语义（探针 mj_forward，无步进）

设置 trunk 偏航 +90°，`qvel[0:3]=[1,0,0]`，`qvel[3:6]=[0,0,0.5]`：`mj_objectVelocity(mjOBJ_XBODY, trunk, local)` = 角 `[0,0,0.5]`、线 `[0,-1,0]`。结论：**free joint `qvel[0:3]` 是世界系线速度，`qvel[3:6]` 是机体系角速度**。上游变量名 `root_lin_vel_b` 不可信；策略未使用它，故不影响策略，但 /odom 必须自行旋转到机体系。第一次探针误用 `mjOBJ_BODY`（惯性/质心系，差 0.055 m 质心偏置与 ~0.009 rad 倾斜）导致两项断言失败，已保留 `c1-probe-run1-failed-objtype.json`。
`framequat` 传感器 == `qpos[3:7]`，`gyro` 传感器 == `qvel[3:6]`：MJCF 中 `noise=0.001/0.005` 只是模型属性，**MuJoCo 不自动加噪**。故 IMU 若发布即为“无噪真值”。

## 5. 模型组合与场景附加

`MujocoController.__init__` 只读 `cfg.robot.mjcf_path`。可选：(a) 用 `MjSpec.from_file` 加载机器人与场景并 `attach`/编译后 `to_xml` 生成组合 MJCF，再以 `cfg.robot.replace(mjcf_path=...)` 注入；(b) 子类改写构造。MuJoCo 3.15.0 中 `MjSpec.from_file/attach/compile` 均存在。MJCF 已自带 `ground` 平面（group 0，contype/conaffinity 1）和两盏灯，场景附加不得再加第二个地面。
几何分组：机器人 25 个视觉 mesh 为 group1、contype/conaffinity 0；22 个碰撞几何为 group0、1/1；世界仅 ground（group0）。射线排除机器人时不能仅靠 geomgroup（与场景同在 group0），W08 须给场景或机器人重新分组，或按 `body_rootid` 过滤。

## 6. 依赖 qpos[7:] / qvel[6:] / ctrl 全长的代码点

全部位于 `mujoco_controller.py`：26–32（qpos 拼接，需 nq=30）、42/126（ghost qpos 全长，需 nq）、120（`set_reference_qpos` 检查 nq）、162–169（update_state 切片）、164/205（`qfrc_actuator[6:]`）、199–205（log_states 切片）、227–228 与 245–246（ctrl_step 切片）、239（`ctrl=` 全数组，需 nu=23）、279（viewer lookat `qpos[0:3]`）。`tasks/` 与 `base_controller.py` 不直接索引 MjData。任何新增 free/hinge 关节或执行器都会破坏这些点；动态障碍须为 mocap body（不增加 nq/nv/nu）。

## 7. 本地 ROS/Nav2 事实（只读）

- Nav2 1.3.13（Jazzy）：`nav2_util::TwistPublisher/TwistSubscriber` 声明 `enable_stamped_cmd_vel`，**默认 false → geometry_msgs/Twist**。
- `nav2_bringup/launch/navigation_launch.py`：controller_server 与 behavior_server 重映射 `cmd_vel→cmd_vel_nav`；velocity_smoother 订阅 `cmd_vel_nav` 发布 `cmd_vel_smoothed`；collision_monitor 默认 `cmd_vel_smoothed→cmd_vel`、`base_frame_id: base_footprint`。
- rclcpp `ClockQoS`：KeepLast(1)、best effort；rclpy time_source 同；SensorDataQoS：KeepLast(5)、best effort。rclpy 执行器无 `spin_some`。
- 已安装：slam_toolbox 2.8.5、nav2_amcl/map_server/mppi/collision_monitor/velocity_smoother 1.3.13、robot_state_publisher、diagnostic_updater。**未安装** twist_mux、pointcloud_to_laserscan、laser_filters（若需要须由用户决定安装，或自研）。
- 无 DISPLAY 时 `import mujoco.viewer` 成功（仅导入，不代表可渲染）。

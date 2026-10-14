# 系统架构（architecture v1）

状态：W02 首版（2026-10-13）。读者：实现 W03–W15 的 agent。接口细节见[interfaces.md](interfaces.md)，验收协议见[acceptance.md](acceptance.md)。变更须先写 ADR（[决策记录](../plans/t1_mujoco_navigation/决策记录.md)）。

## 1. 目标与不变量

Booster T1 在 MuJoCo 中由官方 `t1_walk` 策略和接触动力学行走；仿真 LiDAR 建图/定位；Nav2 从 A 到 B 并对动态障碍绕行或安全等待。全系统不变量：

1. **一个进程、一个线程拥有并推进 MjData**（`t1_sim` 的仿真线程）。其他线程/进程不调用 `mj_step/mj_forward`、不写 `ctrl/qpos/qvel/mocap_*`。
2. **唯一时间源**是 `data.time`，经 `/clock` 发布；同一 ROS 图生命周期内单调。
3. **唯一命令末端**：W10 起 `/cmd_vel` 只有 collision_monitor 发布；t1_sim 末端保护只减不增。
4. **唯一 map→odom 发布者**：mapping 模式 slam_toolbox，navigation 模式 AMCL；模拟器永不发布。
5. **真值只用于理想里程计（/odom 与 odom→base_footprint）和评测**；规划、代价地图、定位不读取障碍物真值或场景脚本。
6. 不修改上游仓库；不改变策略 dt/decimation/PD/观测约定；不以设置基座位姿推进机器人。

## 2. 包与目录职责

| 包 | 构建类型（W03 固定） | 内容 |
| --- | --- | --- |
| `t1_mujoco` | ament_python | `t1_sim` 节点：上游适配子类、索引映射、控制循环、输入缓存、末端保护、command_map 加载、状态机、ROS 发布、LiDAR（W08）、场景组装（W07）、动态障碍事件器（W12）、run 原始记录；`config/command_map.yaml`（W05）；`worlds/`（W07） |
| `t1_description` | ament_cmake | 包装 URDF/xacro：引用 `T1_23dof.urdf`（23 个可动关节，根 `trunk`），加 `base_link→trunk` identity 与 `trunk→lidar_link` 固定关节；RSP 配置；RViz 配置；footprint 多边形（W07） |
| `t1_navigation` | ament_cmake | slam_toolbox、AMCL/map_server、velocity_smoother、collision_monitor、Nav2 参数；地图与 `*.meta.yaml`（W09） |
| `t1_bringup` | W03 决定（若承载评测 Python 则 ament_python） | 顶层 launch（`mode:=sim_only|mapping|navigation|evaluation`）、目标发送、`t1_evaluator`、批量入口（W13） |

运行数据写入 `$WS/run_data/<run_id>/`（已被 .gitignore 忽略）；轻量摘要复制到 `docs/evidence/Wxx/` 或 `docs/results/` 追踪。场景组合生成的展开 MJCF 放 `$WS/run_data/cache/scenes/`，记录来源哈希。

## 3. 上游复用策略（D2）

`t1_sim` 中的 `T1SimRunner` **有限子类化** `booster_deploy.controllers.mujoco_controller.MujocoController`：

| 上游成员 | 处理 | 理由 |
| --- | --- | --- |
| `BaseController.__init__/start/policy_step/stop`、`BoosterRobot`、`RobotData`、`VelocityCommand`、`LocomotionPolicy`（经 `cfg.policy.constructor`） | 原样复用 | 保持策略观测、历史、动作映射、复位语义与冻结版本完全一致 |
| `MujocoController.__init__` | 复用；通过深拷贝后的任务配置 `cfg.robot = cfg.robot.replace(mjcf_path=<组合场景 MJCF>)` 注入场景 | 上游唯一模型注入点；不复制构造代码。前提 nq=30/nv=29/nu=23（interfaces §8 启动断言） |
| `update_state()` | 覆盖：按名字解析的地址数组读取；`root_lin_vel_b` 用 R(q)ᵀ·v_w 正确旋转（策略不用，但避免下游误用） | 去掉隐式切片依赖；修正命名语义 |
| `ctrl_step()` | 覆盖：同一 PD 公式与 effort_limit 裁剪，按 actuator id 写 ctrl；子步钩子（动态障碍 mocap 更新、接触事件）；不读 stdin | 必须插入子步钩子；保持控制律逐项一致（W04 用数值对照证明） |
| `update_vel_command()` | 覆盖为空操作 | 取消 stdin；命令由末端保护写入 |
| `run()` | 不调用；`T1SimRunner.loop()` 自实现（§5） | 上游 run 含 viewer、固定 sleep、stdin，无法 headless、无 ROS |
| `log_states`、ghost 渲染 | 不使用 | 非必要；ghost 不参与感知 |

导入方式：仿真进程内按参数 `upstream_path` 插入 `sys.path`，只导入 `tasks.locomotion.robots.t1`（注册 `t1_walk`）而非遍历全部任务；不依赖 `cd` 上游与 `sys.path.append(".")`；不改 `.bashrc`（W03）。注册表返回全局配置对象，必须 `deepcopy` 后再修改。

## 4. 进程与线程模型

```text
t1_sim 进程
├─ 仿真线程（主线程）  —— 唯一 MjData 所有者
│    模型/策略/末端保护/物理/传感器/场景事件/发布/记录/viewer.sync()
├─ ROS 执行器线程      —— SingleThreadedExecutor.spin()
│    订阅回调：只写 InputBuffer（threading.Lock 保护：最新 Twist + 接收墙钟 + 接收时 sim_time 快照）
│    服务回调：放入 RequestQueue，等待仿真线程在步边界处理后 set Event（墙钟 2 s 超时返回 success=false）
└─ viewer 渲染线程（仅 GUI 模式，mujoco.viewer.launch_passive 自带）
     只在仿真线程调用 viewer.sync() 时与 MjData 同步；键盘回调只置标志位
```

- rclpy 执行器没有 `spin_some`，因此用独立执行器线程；发布在仿真线程调用（rclpy publish 线程安全）。
- GUI 模式下修改 MjData 的代码块遵循 MuJoCo 3.15 passive viewer 的线程约定（`viewer.lock()` 或仅在 sync 之间修改）；W04 读取本地 `mujoco/viewer.py` 确认并记录。
- LiDAR 首版在仿真线程同步计算。若 W08/W13 测得超预算，只允许在**快照副本**（`mj_copyData` 到独立 MjData）上于工作线程计算，并以快照时刻打 stamp；绝不读写活动 MjData。
- 外部进程（Nav2、SLAM、evaluator、teleop）只通过 ROS 话题/服务交互。

## 5. 控制周期（每策略步 k，`data.time` 从 t_k 推进到 t_k+0.02）

| 序 | 动作 | 说明 |
| --- | --- | --- |
| 0 | 节拍等待 | `target_rtf>0` 时按单调墙钟 deadline 等待（deadline += policy_dt/target_rtf；落后则重置 deadline 并计数，不突发追赶）；`0` 不等待 |
| 1 | 处理请求 | pause/resume/reset 在此边界生效（interfaces §7.3），然后 set 服务 Event |
| 2 | 状态检查 | 非 STARTING/READY（PAUSED/FALLEN/FAULT）→ 只做心跳与 viewer 同步，回到 0 |
| 3 | 命令锁存与末端保护 | 取 InputBuffer 快照 → interfaces §6.3 第 1–8 条 → 写 `VelocityCommand`；STARTING 期间强制 0 |
| 4 | `update_state()` | 从当前 MjData 读取（按名地址） |
| 5 | `policy_step()` | 推理得目标；检查目标有限；若 `controller.is_running` 被策略置 False（跌倒判据）→ FALLEN，**不施加本步动作** |
| 6 | 10 个物理子步 | 每子步：场景事件器按 `data.time` 写 mocap（W12）→ `ctrl = clip(kp(q*−q) − kd·q̇, ±effort_limit)` → `mj_step` → 接触事件采样（W12/W13） |
| 7 | 有限性检查 | qpos/qvel 非有限 → FAULT `nonfinite_state`，停止推进 |
| 8 | 位置缓存刷新 | 本步需要 xpos 派生量（扫描、真值体位姿）时调用 `mj_kinematics`，使几何缓存对应当前 `data.time`（mj_step 后缓存滞后一个物理步） |
| 9 | 发布 | `/clock` → 同 stamp 的 /odom、TF、/joint_states（按分频）、真值 → 到期的 /scan（k mod 5 == 0）→ trace CSV 行 |
| 10 | 心跳 | 墙钟 1 Hz：`/t1/sim_state`、`/diagnostics`、RTF |
| 11 | viewer | GUI 模式按 `viewer_rate_hz`（墙钟，低于物理频率）`viewer.sync()` |

上游 `run()` 的差异：上游每轮先 `sleep(0.02)` 再计算（使 RTF<1），并在 `ctrl_step` 内读命令（晚一拍）；本项目改为 deadline 节拍与步首锁存。W04 须用相同初态与命令序列对照两者轨迹，差异 >10% 先解释。

## 6. 数据流

```text
                 ┌───────────── t1_sim（唯一 MjData）──────────────┐
 /t1/debug/      │ InputBuffer → 末端保护 → M(v) → VelocityCommand │
 policy_cmd ───► │        ▲ (W04/W05 实验)            │           │
                 │        │                           ▼           │
 /cmd_vel ──────►│  双重超时/包络/只减不增     LocomotionPolicy     │
   ▲             │                                   │ 目标       │
   │             │   场景事件器(mocap) ──► 物理 10×mj_step ◄─ PD   │
   │             │                                   │            │
   │             │  /clock /odom /tf /joint_states /scan /t1/sim_state /diagnostics
   │             │  /t1/eval/ground_truth   run_data/<run_id>/*.csv │
   │             └──────┬──────────┬──────────┬───────────┬────────┘
   │                    │/scan     │/tf,/odom │/joint_states│真值
   │                    ▼          ▼          ▼             ▼
   │   slam_toolbox XOR amcl ─(map→odom)─► TF 树 ◄─ robot_state_publisher   t1_evaluator
   │                    │/map                │
   │                    ▼                    ▼
   │      planner_server ─► bt_navigator ─► controller_server(MPPI) / behavior_server
   │                                              │ /cmd_vel_nav
   │                                              ▼
   │                                      velocity_smoother
   │                                              │ /cmd_vel_smoothed
   │                                              ▼
   └────────────────────────────────────── collision_monitor ◄─ /scan
```

mapping 模式下 teleop 替代 Nav2 核心作为 `/cmd_vel_nav` 源（W10 起；W09 时直接 `/cmd_vel`）。

## 7. 启动模式

| 节点 | sim_only | mapping | navigation | evaluation |
| --- | --- | --- | --- | --- |
| t1_sim | ✓ | ✓ | ✓ | ✓（含场景事件器） |
| robot_state_publisher | ✓（W06 起） | ✓ | ✓ | ✓ |
| slam_toolbox | — | ✓ | — | — |
| map_server + amcl | — | — | ✓ | ✓ |
| velocity_smoother + collision_monitor | — | ✓（W10 起） | ✓ | ✓ |
| Nav2 规划/控制/行为/BT | — | — | ✓ | ✓ |
| teleop | 可选（W05 标定后） | ✓ | — | — |
| t1_evaluator | — | — | 可选 | ✓ |

启动顺序：t1_sim（STARTING，先发 /clock=0）→ RSP → 定位/建图 → 安全链 → Nav2 lifecycle（等待 /clock 前进、`odom→base_footprint` TF、/scan、`map→odom` 可用后激活）→ evaluator 在 `/t1/sim_state.state==READY` 且定位就绪后发目标。原地 reset 只在 sim_only；其他模式每个 run 一次完整 launch。

## 8. 所有权总表

| 对象 | 唯一所有者 | 其他人权限 |
| --- | --- | --- |
| MjData / 物理推进 / mocap 写入 | t1_sim 仿真线程 | 无 |
| `/clock` | t1_sim | 只读 |
| `odom→base_footprint`、`base_footprint→base_link`、`base_footprint→laser_stabilized` | t1_sim | 只读 |
| `base_link→trunk`、关节树、`trunk→lidar_link` | robot_state_publisher | 只读 |
| `map→odom` | slam_toolbox（mapping）/ amcl（navigation） | 只读 |
| `/map` | slam_toolbox（mapping）/ map_server（navigation） | 只读 |
| 最终正常命令 `/cmd_vel` | collision_monitor（W10 起） | t1_sim 只可减小/置零 |
| v_des→u_pol 映射与运动包络 | W05 的 `command_map.yaml` | t1_sim 只读加载；Nav2/smoother 限值须与之一致 |
| 安全区、超时阈值 | W10 冻结的参数文件 | — |
| 场景、A/B、footprint | W07 配置 | — |
| 地图及 `T_world_map` | W09 地图目录 | 场景/评测只读 |
| 评测真值与 run 原始记录 | t1_sim 写、t1_evaluator 读 | 导航栈禁止读取 |

## 9. W03–W12 实现对照

| 步骤 | 实现的接口/产物 | 拥有 |
| --- | --- | --- |
| W03 | 四包骨架；`mode`/`use_sim_time` launch 参数管线；参数文件位置；解释器与 `upstream_path` 解析；诊断入口 | 构建与启动环境 |
| W04 | `T1SimRunner`（§3）、索引映射与启动断言、控制周期（§5）、InputBuffer、末端保护 1–3/7/8（无标定时 UNCALIBRATED 拒绝 /cmd_vel）、`/t1/debug/policy_cmd`、`/clock`、`/t1/sim_state`、`/diagnostics`、pause/reset 服务、状态机、headless/viewer、`run_data` manifest 与基础 trace | MjData、/clock、t1_sim 状态 |
| W05 | `command_map.yaml`（M、envelope、deadband、能力开关）、末端保护 4–6 及单元测试、warmup、停距/停时、终点容差与时间上限修订 | 速度契约 |
| W06 | `/odom`、`odom→base_footprint→base_link` TF、`/joint_states`、RSP 基础包装 URDF（base_link→trunk）、可选 `/imu/data`、`/t1/eval/ground_truth` | 动态 TF、状态桥 |
| W07 | 场景组合（MjSpec，缓存展开 MJCF）、W07 单步文件所列场景（91dd117 起：几何校准场、static_world、dynamic_crossing_world、stress_test_world 静态布局与障碍生成接口）及上游场景来源/许可证清单、A/B、包装 URDF 完整化、footprint、射线几何分组 | 场景与描述 |
| W08 | `/scan`（frame `laser_stabilized`）、`base_footprint→laser_stabilized` TF、可选 `/t1/lidar/points`、扫描健康门、`sensor_mode` 选择 ADR | 传感器 |
| W09 | slam_toolbox mapping、地图保存与 `*.meta.yaml`（含 `T_world_map`、场景/代码 SHA）、map_server+AMCL、`/initialpose` 流程 | map→odom（按模式）、地图 |
| W10 | velocity_smoother、collision_monitor、命令链接线与源互斥、超时冻结、安全区、故障注入 | 最终命令通道 |
| W11 | Nav2 规划/MPPI/BT/costmap/goal checker、navigation launch、目标发送入口、静态 5 例 | 导航参数 |
| W12 | t1_sim 内动态障碍事件器（mocap）、三类场景配置、接触事件记录 | 动态障碍 |

## 10. 一致性审查（W02 C4）

| 检查 | 结论 | 依据/残余风险 |
| --- | --- | --- |
| /clock 双发布 | 无：仅 t1_sim | acceptance §8 检查发布者数 = 1 |
| TF 同一 child 多发布者 | 无：§8 每个 child 一个发布者；map→odom 按模式互斥 | 用 `tf2_monitor` 验证每帧 authority；`base_link` 与 `trunk` 是两个 child，不重复 |
| `/map` 双发布 | 无：slam_toolbox 与 map_server 按模式互斥 | — |
| `/cmd_vel` 多源竞争 | W10 起仅 collision_monitor；teleop 与 Nav2 按模式互斥 | `/cmd_vel_nav` 在 navigation 模式有 controller_server 与 behavior_server 两个发布者，是 Nav2 设计（BT 串行激活），不是竞争；teleop 不得同时运行 |
| `/odom` 双发布 | 无：仅 t1_sim；不启用 robot_localization 等第二估计器 | — |
| TF 环路 | 无：树形，每 child 单父；`mj_world` 不入树 | — |
| 命令环路 | 无：命令链线性；smoother 闭环只读 /odom，是反馈而非话题环 | — |
| 重复状态真相 | 无：MjData 是唯一状态源，/odom（平面理想里程计）与 /t1/eval/ground_truth（6D 评测真值）是同源两种视图，用途隔离；时间唯一 /clock；`VelocityCommand` 只由仿真线程写 | 理想里程计无漂移，须在报告披露 |
| 隐含依赖 | 已显式化：①上游 nq/nv/nu 假设→启动断言；②Nav2 stamped 默认值→显式参数；③costmap 默认 `robot_base_frame: base_link`→统一改 base_footprint；④地面 z=0→W07 约束；⑤teleop 默认发 /cmd_vel→按模式重映射；⑥Nav2 默认 BT 含 spin/backup→W11 按 W05/W10 能力禁用；⑦collision_monitor 默认 FootprintApproach 依赖 `/local_costmap/published_footprint`→W10 配置；⑧上游 `sys.path.append(".")`→W03 参数化路径；⑨注册表全局配置→深拷贝；⑩/clock 50 Hz 与 Nav2 计时器粒度→W06/W11 实测，必要时提频（ADR）；⑪twist_mux/pointcloud_to_laserscan/laser_filters 未安装→不依赖，需要时由用户决定安装 | 后续步骤发现新的隐含依赖须补到本表 |

## 11. 备选方案与未采用原因

| 方案 | 未采用原因 |
| --- | --- |
| 直接修改上游 booster_deploy（在 MujocoController 内加 ROS、改切片） | 破坏 W01 冻结 SHA 与可追溯性，违反执行规则“上游保持原 SHA/clean”；差异难审计、难同步上游。改为有限子类覆盖 4 个方法 |
| 多线程共同写 MjData（ROS 回调线程直接写 ctrl/mocap 或 mj_step；传感器线程直接在活动 MjData 上射线） | MjData 非线程安全，GIL 不保护 C 层复合操作；会产生物理/传感器时间不一致和不可复现结果。改为单所有者 + 输入缓存 + 必要时快照副本 |
| 纯真值地图导航（MJCF 直接栅格化成地图、规划器读障碍真值、模拟器发 map→odom） | 违反 R2/R3/D4：LiDAR 必须是地图和障碍感知来源，须展示建图与定位；会让动态避障演示失真 |
| ros2_control + mujoco_ros2_control 硬件接口 | 本机未安装，需新依赖；50 Hz Python/torch 策略需自写控制器插件；范围过大，收益只是标准化 |
| 独立传感器进程（共享内存读物理状态） | 跨进程快照一致性与时间戳复杂；首版同步计算，必要时进程内快照副本 |
| TwistStamped 全链路 | Jazzy Nav2 1.3.13 与 teleop 默认无时间戳；混发风险高。命令新鲜度改用 t1_sim 接收时刻（双重超时） |
| twist_mux 选择命令源 | 未安装；按 launch 模式互斥即可满足“唯一活动源” |
| 运动学滑动基座（直接设 qpos 跟随命令） | 违反 R1，明确禁止 |

## 12. 交给 W05 的能力门槛（本架构不作承诺）

最小可持续前进速度与死区、最大前进速度、偏航率范围、前进+转弯组合、倒退、原地旋转、横移、起步/停止延迟、零命令停距/停时与稳态判据、warmup 时长、终点位置/偏航容差与 120 s 上限是否需修订。任何一项不可行时：在 `command_map.yaml` 中禁用并让 W10/W11 的限值、恢复行为和运动模型随之收紧；若无可行包络，W05 BLOCKED（G1），不由导航层掩盖。

## 13. W10实施细化（ADR-014）

正常sim_only/mapping/localization启动均含t1_command_gate、velocity_smoother、collision_monitor。所选原始源→gate→/cmd_vel_nav→smoother→/cmd_vel_smoothed→CM→/cmd_vel；gate额外发布仅供末端否决的permit，不生成第二个最终速度。只有显式sim_only policy debug隔离安全链。实际恢复协议与时延/区域推导见[safety.md](safety.md)。W11 controller/behavior须经单一导航源适配器进入/t1/cmd_navigation（本步仅接收一个发布者），并把取消接到stop服务；不能直接重映射两个发布者到最终话题。

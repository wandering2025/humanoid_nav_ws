# 接口契约（interfaces v1）

状态：W02 冻结的首版契约（2026-10-13）。读者：实现 W03–W15 的 agent。改动任何一行先写[决策记录](../plans/t1_mujoco_navigation/决策记录.md)ADR，并列出需重验的步骤；W06/W08/W10 等可在本文件“由 Wxx 细化”标注处填入实测值，不得改变语义。
配套文档：[system.md](system.md)（进程、线程、控制周期、所有权）、[acceptance.md](acceptance.md)（验收协议）。上游事实来源：[c1-source-review.md](../evidence/W02/c1-source-review.md)、[c1-probe.json](../evidence/W02/c1-probe.json)。

约定：ROS 2 Jazzy；REP-103 单位与坐标（m、rad、s；x 前、y 左、z 上）；REP-105 frame 名称语义。“待W05确定”表示数值未测，禁止填写猜测值当事实。`t1_sim` 指 t1_mujoco 包中唯一拥有 MjData 的仿真节点。

## 1. 节点清单

| 节点 | 包 | 首次实现 | 启动模式 | 职责 |
| --- | --- | --- | --- | --- |
| `t1_sim` | t1_mujoco | W04（W06/W08/W12 扩展） | 全部 | 唯一 MjData 所有者：物理、策略、命令末端保护、/clock、状态与传感器发布、场景事件、评测原始记录 |
| `robot_state_publisher` | 系统包，配置在 t1_description | W06/W07 | 全部（W06 起） | /robot_description、/tf_static、关节树 /tf |
| `slam_toolbox` | 系统包，配置在 t1_navigation | W09 | mapping | 建图，唯一 map→odom 发布者 |
| `map_server` + `amcl` | 系统包，配置在 t1_navigation | W09 | navigation | 加载地图；AMCL 为唯一 map→odom 发布者 |
| `velocity_smoother` | nav2 | W10 | mapping（W10 起）、navigation | /cmd_vel_nav → /cmd_vel_smoothed |
| `collision_monitor` | nav2 | W10 | mapping（W10 起）、navigation | /cmd_vel_smoothed → /cmd_vel，唯一正常最终命令发布者 |
| planner/controller(MPPI)/behavior/bt_navigator 及 lifecycle manager | nav2 | W11 | navigation | 规划与跟踪；controller/behavior 发布 /cmd_vel_nav |
| `teleop_twist_keyboard`（或同类） | 系统包 | W09 | mapping、sim_only 手动 | 人工命令源；与导航互斥 |
| `t1_evaluator` | t1_bringup | W13（W11/W12 可最小版） | evaluation | 读取评测真值与 Nav2 结果，写结果文件；不发布任何命令/TF/地图 |

不使用自定义 msg/srv 类型（四包结构不含 rosidl 包）。场景/动态障碍事件器是 `t1_sim` 内部模块，不是独立节点，不能从外部进程写 MjData。

## 2. 话题

QoS 缩写：R=Reliable，BE=BestEffort，V=Volatile，TL=TransientLocal，KL(n)=KeepLast(n)。仿真发布者一律用 R（与 BE 订阅者兼容）；BE 订阅者能连 R 发布者，反之不行。RViz 订阅传感器话题时设 Best Effort 或 Reliable 均可。

| 话题 | 类型 | 生产者 | 消费者 | frame | 时间戳 | 频率（仿真时间） | 发布 QoS | 失效行为 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `/clock` | rosgraph_msgs/Clock | **仅** t1_sim | 所有 use_sim_time 节点 | — | `data.time` | 每策略步 50 Hz；W06/W11 可改为每 1/2/5 物理步（≤500 Hz），须 ADR | R, V, KL(1) | 暂停/跌倒/故障时停止发布，消费者计时器冻结；永不回跳（§5） |
| `/odom` | nav_msgs/Odometry | t1_sim（W06） | velocity_smoother（若闭环）、controller_server、slam_toolbox/AMCL 不直接订阅（经 TF） | header `odom`，child `base_footprint` | 采样时 `data.time` | 50 Hz | R, V, KL(10) | 停止发布即视为 odom 丢失；W10 定义下游停车 |
| `/tf`（odom→base_footprint、base_footprint→base_link、base_footprint→laser_stabilized（W08 起）） | tf2_msgs/TFMessage | t1_sim（W06/W08） | 全体 | 见 §4 | 与同一步 /odom 完全相同 | 50 Hz | tf2 默认 | 同上 |
| `/tf`（关节树）、`/tf_static` | tf2_msgs/TFMessage | robot_state_publisher | 全体 | 见 §4 | 来自 /joint_states stamp | 跟随 /joint_states | tf2 默认；static 为 TL | — |
| `/joint_states` | sensor_msgs/JointState | t1_sim（W06） | robot_state_publisher、记录 | — | `data.time` | 50 Hz（可配置为策略频率整数分频，如 25 Hz） | R, V, KL(10) | 停止发布 → RViz 模型冻结，不影响导航 |
| `/scan` | sensor_msgs/LaserScan | t1_sim（W08） | slam_toolbox、AMCL、local/global costmap、collision_monitor、RViz | 恒为 `laser_stabilized`（D5 两种生成方式共用：机体固定多束点云去倾斜 `compensated`，或有标注的降级 `ideal_stabilized`；W08 选择并 ADR，写入 manifest `sensor_mode`）。不发布随躯干倾斜的平面扫描（会破坏 2D SLAM） | 射线采样时 `data.time` | 初始 10 Hz（每 5 策略步） | R, V, KL(5) | 生成异常 → 不发布该帧 + diagnostics ERROR + t1_sim 内部健康门置零命令；暂停时不产生扫描 |
| `/t1/lidar/points`（可选） | sensor_msgs/PointCloud2 | t1_sim（W08） | RViz、调试 | `lidar_link` | 同 /scan | 同 /scan | R, V, KL(2) | 可关闭；导航不得依赖 |
| `/imu/data`（首版**不发布**） | sensor_msgs/Imu | t1_sim（W06 若决定发布） | 无必需消费者 | `base_link` | `data.time` | 50 Hz | R, V, KL(10) | 若发布：orientation/angular_velocity 来自 MJCF `framequat`/`gyro`，实测无自动噪声→为**仿真真值**，须在 diagnostics 与文档标注；MJCF 无加速度计→`linear_acceleration_covariance[0]=-1`，值填 0；不得给未测字段填零协方差 |
| teleop 输出（不单独成话题） | geometry_msgs/Twist | teleop | 经 launch 重映射：W09（W10 前）直接到 `/cmd_vel`；W10 起到 `/cmd_vel_nav`；navigation/evaluation 模式不启动 teleop | base_footprint 语义 | 无 | 源决定 | R, V, KL(10) | — |
| `/cmd_vel_nav` | geometry_msgs/Twist | controller_server、behavior_server（navigation）**或** teleop（mapping，W10 起），二者互斥 | velocity_smoother | base_footprint 语义（物理期望速度） | 无 | 控制器 20 Hz（W11 定） | R, V, KL(1) | 源停止 → smoother `velocity_timeout` 后发零 |
| `/cmd_vel_smoothed` | geometry_msgs/Twist | velocity_smoother | collision_monitor | 同上 | 无 | smoother 频率（W10 定） | R, V, KL(1) | — |
| `/cmd_vel` | geometry_msgs/Twist | **W10 起仅** collision_monitor；W04–W09 为受控开发源（teleop/脚本） | t1_sim 末端保护 | 同上 | 无 | 跟随上游 | 订阅 R, V, KL(1) | t1_sim 双重超时（§6.3）→ 零命令继续站立 |
| `/t1/debug/policy_cmd` | geometry_msgs/Twist | W04/W05 实验脚本 | t1_sim | **策略单位**，非物理量 | 无 | 实验决定 | R, V, KL(1) | 仅当参数 `enable_policy_cmd_input:=true` 时订阅，此时 `/cmd_vel` 被忽略；导航/评测 launch 禁止开启 |
| `/t1/sim_state` | std_msgs/String（JSON，§7.2） | t1_sim | evaluator、bringup 就绪检查、记录 | — | JSON 内含 sim_time | 状态变化时 + 墙钟 1 Hz | R, **TL**, KL(1) | 进程退出即停止；消费者以墙钟 3 s 无更新判 STALE |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | t1_sim、Nav2 节点 | rqt/记录/evaluator | — | t1_sim 用当前 `data.time`（暂停时冻结）；其他节点为各自 sim 时间 | 墙钟 1 Hz + 变化时 | 默认 | — |
| `/t1/eval/ground_truth` | nav_msgs/Odometry | t1_sim | **仅** t1_evaluator、记录器 | header `mj_world`，child `base_link_truth`（均不在 TF 树） | `data.time` | 50 Hz | R, V, KL(10) | 导航/定位/代价地图节点禁止订阅（acceptance §8 检查） |
| `/map`、`/map_metadata` | nav_msgs/OccupancyGrid 等 | slam_toolbox（mapping）**或** map_server（navigation） | costmap、RViz | `map` | 生产者 | 生产者 | TL | — |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 用户/RViz/bringup 一次性初始化 | AMCL | `map` | — | 一次 | 默认 | 不得周期性用真值重置（W09） |
| `/collision_monitor_state` | nav2_msgs/CollisionMonitorState | collision_monitor | 记录、evaluator | — | — | — | 默认 | — |

明确不存在的话题：任何把障碍物真值或场景脚本未来轨迹提供给规划/代价地图的话题；模拟器发布的 map→odom；第二个 /clock 发布者。若为 RViz 演示需要障碍物真值标记，只能用 `/t1/viz/*` MarkerArray，且 acceptance §8 检查无 Nav2 节点订阅。

## 3. 服务与关键参数

| 服务 | 类型 | 服务端 | 语义 |
| --- | --- | --- | --- |
| `/t1/sim/pause` | std_srvs/SetBool | t1_sim | true=暂停、false=恢复；在下一个策略步边界生效后才返回 success |
| `/t1/sim/reset` | std_srvs/Trigger | t1_sim | 仅 `mode=sim_only` 允许（§7.3）；其他模式 `success=false`，消息说明须重启整套 launch |

t1_sim 参数（具体文件位置由 W03 固定）：`upstream_path`、`task`（固定 `t1_walk`）、`scene`、`mode`（`sim_only|mapping|navigation|evaluation`）、`headless`、`viewer_rate_hz`、`target_rtf`、`command_map_file`、`enable_policy_cmd_input`、`cmd_timeout_sim`、`cmd_timeout_wall`、`warmup_sim_s`、`run_id`、`run_dir`、`seed`、各发布分频。`policy_dt=0.02`、`decimation=10`、`physics_dt=0.002` 来自上游任务配置，**不是**可调参数；启动时校验并写入 manifest。

Nav2 全部节点显式设置 `enable_stamped_cmd_vel: false`（本机 Nav2 1.3.13 默认即 false，但不依赖默认值）；teleop 设 `stamped:=false`。所有平面消费者的机器人 frame 统一为 `base_footprint`：costmap `robot_base_frame`、collision_monitor `base_frame_id`、AMCL `base_frame_id`、slam_toolbox `base_frame`。除 t1_sim 外所有节点 `use_sim_time: true`。

## 4. Frame 与 TF 所有权

```text
map ──(slam_toolbox XOR amcl)──> odom ──(t1_sim)──> base_footprint ──(t1_sim)──> base_link
                                                         │                         │ (RSP static, identity)
                                                         │                         v
                                         (t1_sim, W08 起)                         trunk ──(RSP)──> 关节树 links
                                                         v                         │ (RSP static)
                                                  laser_stabilized                 v
                                                                               lidar_link
mj_world / base_link_truth：仅评测真值，不进入 TF 树
```

| child | parent | 唯一发布者 | 动/静 | 定义 |
| --- | --- | --- | --- | --- |
| `odom` | `map` | slam_toolbox（mapping）或 amcl（navigation） | 动 | 两者由 launch 模式互斥；无定位器时（W04–W08）不存在 map，RViz 固定 frame 用 `odom`；禁止用静态 identity 掩盖 |
| `base_footprint` | `odom` | t1_sim | 动 50 Hz | 原点 = trunk 机体系原点在 z=0 地面上的竖直投影；x 轴 = trunk 偏航（`yaw=atan2(2(wz+xy),1-2(y²+z²))`，MuJoCo wxyz）；z 竖直向上；roll=pitch=0，z=0 |
| `base_link` | `base_footprint` | t1_sim | 动 50 Hz | **与 MuJoCo `trunk` 机体系（mjOBJ_XBODY，free joint 帧，`qpos[0:7]`）完全重合**：平移 (0,0,z_trunk)，旋转 = R_yaw(ψ)ᵀ·R_trunk（只含 roll/pitch）。不是质心/惯性系 |
| `trunk` | `base_link` | robot_state_publisher（t1_description 包装 URDF 的固定 identity 关节） | 静 | URDF `T1_23dof.urdf` 根 link 名为 `trunk`；用别名关节连接，避免改上游 URDF |
| 关节树 links | 父 link | robot_state_publisher | 动 | 由 /joint_states 驱动；W07 用 FK 核对与 MJCF 一致 |
| `lidar_link` | `trunk` | robot_state_publisher（包装 URDF 固定关节） | 静 | 刚性安装在 trunk（不装头部：头部关节受 PD 误差影响）；位姿 W08 定 |
| `laser_stabilized` | `base_footprint` | t1_sim（W08） | 动 50 Hz | 原点 = lidar_link 原点在 base_footprint 中的位置（随 roll/pitch 变化），姿态与 base_footprint 相同（重力对齐、同偏航） |

`odom` 原点与 MuJoCo 世界系水平投影重合（理想里程计 = 真值投影；A2/D4）。因此 odom 无漂移，这一点必须在最终报告披露。`map` 与 `mj_world` 的关系 `T_world_map` 由 W09 在建图起点从真值记录进地图元数据（`<map>.meta.yaml`），只供场景配置换算 A/B 和评测使用，导航节点不读取。
场景地面必须是 z=0 的平面（MJCF 自带 `ground`）；W07 不得改变地面高度。

## 5. 时间

- **唯一时间源**：MuJoCo `data.time`。t1_sim 自身 `use_sim_time=false`，但所有发布消息 stamp 显式用 `data.time`；不用 `node.get_clock().now()` 打仿真消息时间戳。
- 每策略步顺序：完成 10 个物理步 → 发布 `/clock=data.time` → 发布同一时刻派生的 odom/TF/joints（以及到期的 scan）。同一步派生的消息 stamp 完全相同。启动时在第一步之前先发布一次 `/clock=0`。
- t1_sim 内部所有“按仿真时间调度”的事件（扫描、分频发布、warmup、场景事件）以策略步计数决定，不创建依赖 /clock 的 ROS 定时器（避免启动死锁）。
- 墙钟（`time.monotonic()`）只用于：命令墙钟超时、实时节拍（deadline 方式，W04）、RTF 统计、diagnostics/状态心跳、viewer 刷新。墙钟不出现在任何消息 stamp 中。
- 实时节拍：`target_rtf`（默认 1.0；headless 批量可设 `0` 表示不限速）。落后时不跳过物理步、不追赶突发；记录超时计数与 RTF。不得通过伪造 /clock 宣称实时。
- **时间永不回跳**：同一 ROS 图生命周期内 /clock 单调不减。原地复位保持单调（§7.3）；完整复位 = 重启整套进程（新 run_id，新图）。消费者若观测到回跳，该 run 判无效。

## 6. 速度命令语义

### 6.1 三种量（代码、CSV、文档统一用这些名字）

| 名称 | 含义 | 单位/frame | 出现位置 |
| --- | --- | --- | --- |
| `v_des` | 物理期望速度 | m/s、rad/s；base_footprint（vx 前、vy 左、wz 逆时针） | `/cmd_vel_nav`、`/cmd_vel_smoothed`、`/cmd_vel` |
| `u_pol` | 策略输入命令（观测中的 cmd 三维原值） | 策略单位，**不是** m/s；上游 `VelocityCommandCfg` 允许 ±1.0 | t1_sim 内部 `VelocityCommand` 字段；`/t1/debug/policy_cmd`；CSV |
| `v_meas` | 实测速度 | m/s、rad/s；base_footprint | `/odom.twist`；CSV |

已知历史单点：u_pol=0.5 → 实速约 0.39 m/s；0.2/0.3 不能持续行走（SETUP_REPORT §16，未复测）。禁止把任何 u_pol 数值当作 v_des 或 v_meas 使用，禁止把单点比例外推为线性增益。

### 6.2 映射 M：v_des → u_pol

- 归属：W05 产出 `src/t1_mujoco/config/command_map.yaml`（生产者 W05，消费者 t1_sim 末端保护）。字段：`schema`、`calibrated`（bool）、`evidence`（W05 证据路径与被测 SHA）、`envelope`（允许的 vx/vy/wz 范围及组合约束）、每轴单调分段线性表 `(v_phys, u_pol)`、`deadband` 规则、`reverse_allowed`、`lateral_allowed`、`in_place_turn_allowed`、适用条件。数值由 W05 确定（ADR-010）：前进 vx∈[0.31,0.75] m/s，0<vx<0.31 置零；行走中 |wz|≤0.45 rad/s；原地（vx=0）|wz|∈[0.545,0.85]，以下置零；倒退/横移禁用。wz 表按模式与符号分为 `wz_walking_pos/neg`、`wz_in_place_pos/neg`（各自单调，M(0)=0）。文件由 docs/evidence/W05/build_command_map.py 生成，依据见 docs/evidence/W05/locomotion-envelope.md。`v_guard` = M⁻¹(u_pol)。
- 约束：M(0)=0 精确；每轴单调不减；只在 W05 实测验证区间内定义；区间外裁剪到边界，不外推。允许数值放大（例如为实现 0.39 m/s 需要 u_pol=0.5），但按 M 预期的物理速度不得超过输入的 v_des（§6.3 第 6 条）。
- `calibrated:false` 或文件缺失时：t1_sim 拒绝 `/cmd_vel`（u_pol=0，`cmd_status=UNCALIBRATED`，diagnostics WARN），只有 `/t1/debug/policy_cmd` 可动（W04/W05 实验用）。navigation/evaluation 模式启动时若未标定则启动失败。
- 自由度预设：只用 vx≥0 与 wz；vy 恒 0。是否允许倒退、原地旋转、横移由 W05 实测决定写入 `envelope`；未验证则禁止（MPPI/smoother 对应限值为 0，behavior_server 不启用 backup，spin 视 W05/W10 结论）。导航运动模型与该动作集一致（W11）。

### 6.3 t1_sim 末端保护（每策略步一次，只减不增）

1. 从输入缓存取最新命令及其接收时刻（墙钟 monotonic 与当时 `data.time`）。
2. 有效性：6 个分量全部有限；`linear.z/angular.x/angular.y` 非零则忽略并计数。非有限 → u_pol=0、`cmd_status=INVALID`。
3. 新鲜度：`age_sim ≤ cmd_timeout_sim` **且** `age_wall ≤ cmd_timeout_wall`，否则 u_pol=0、`TIMEOUT`。初值均 0.5 s，W10 冻结。双条件理由：墙钟条件保证暂停/卡顿期间命令过期且恢复时不复活；仿真条件保证 RTF>1 时不会在仿真中走得比预期远。
4. 包络裁剪（物理单位）：按 `envelope` 裁剪各轴幅值、禁用未验证自由度（置 0）。只能减小幅值。
5. 死区：按 W05 规则只能“置零”或“保持”，**禁止把小于可行最小速度的请求抬升到起步速度**。需要抬升的逻辑（若 W05 证明必要）只能放在 Collision Monitor 之前（控制器/平滑器参数），并写 ADR。
6. 映射 u_pol = M(v)；单元测试断言：每轴 `sign(M⁻¹(u_pol)) = sign(v)` 且 `|M⁻¹(u_pol)| ≤ |v|`。
7. 状态门：t1_sim 状态非 READY，或内部扫描健康门失败 → u_pol=0（`BLOCKED_BY_STATE`）。
8. 写入 `VelocityCommand.lin_vel_x/lin_vel_y/ang_vel_yaw`，同一步生效，随后才 `update_state()`/`policy_step()`。末端无低通、无保持、无缓慢衰减：零或减小在本步立即生效。加速限制只在 velocity_smoother（碰撞过滤之前）。

注意上游原行为：stdin 命令在 `ctrl_step()` 中读取，下一策略步才进观测（晚一拍）。本项目在策略步开始时锁存，零延迟进入观测；与上游的差异写入 W04 适配说明。

### 6.4 站立与停车

“停车/急停” = u_pol=(0,0,0)，策略继续运行保持平衡（站立策略），PD 与力矩正常。**禁止**以全关节力矩置零或阻尼模式当作安全站立。物理制动时间与停距由 W05 测量，W10 用于安全区推导。跌倒（上游判据：投影重力 z > −0.5，即倾角 >60°；W10 可加高度判据）→ `FALLEN`：策略停止，物理不再推进，/clock 停止，run 终止为失败，需要重启；不声称能自动站起。

## 7. 状态机、暂停、复位

### 7.1 t1_sim 状态

`STARTING`（模型校验 + `warmup_sim_s` 零命令站立，W05 冻结 2.0 s）→ `READY` ⇄ `PAUSED`；`READY → FALLEN | FAULT`；`任意 → RESETTING → STARTING`（仅 sim_only）；`SHUTDOWN`。READY 之前收到的命令一律丢弃。
`FAULT` 原因（字符串枚举）：`model_incompatible`（拒绝启动）、`nonfinite_state`、`nonfinite_action`、`policy_exception`、`sensor_exception`、`internal_error`。FAULT 下 u_pol=0；若状态非有限则停止推进物理；无自动恢复，显式复位/重启才离开。

### 7.2 `/t1/sim_state` JSON（schema 1）

`{"schema":1,"run_id":str,"episode":int,"state":str,"fault":str|null,"mode":str,"sim_time":float,"wall_unix":float,"rtf_10s":float,"cmd_status":"ACTIVE|IDLE|TIMEOUT|INVALID|UNCALIBRATED|BLOCKED_BY_STATE","cmd_source":"cmd_vel|policy_cmd","v_des":[vx,vy,wz],"u_pol":[ux,uy,uwz],"scenario":str,"seed":int,"code_sha":str}`。W05 追加（向后兼容）：`v_guard:[vx,vy,wz]`、`guard:[str]`（vx_deadband/wz_deadband/vx_clipped/wz_clipped/vy_disabled/reverse_disabled/in_place_disabled）、`command_map_calibrated:bool`。新增字段向后兼容；改名或改语义须提升 schema 并 ADR。

### 7.3 暂停、复位、时间回跳的清理表

| 事件 | 物理/clock | 命令缓存 | 策略历史（obs_history、last_action） | 传感器调度/缓存 | TF / 下游 |
| --- | --- | --- | --- | --- | --- |
| 暂停 | 不推进；/clock 停发 | 进入与恢复时都清空；恢复后只接受接收时刻晚于恢复时刻的新命令 | 保留（状态被冻结，与历史一致） | 保留下次到期步号；暂停中不产生扫描 | 不发布；下游因 sim time 冻结不会外推 |
| 原地复位（sim_only） | `mj_resetData` 后重新初始化 qpos，并把 `data.time` 设为复位前时间 + policy_dt 保持单调 | 清空 | `policy.reset()`（history=None、last_action=0、filtered_target=default），`start()` 重置步数 | 清空，扫描调度从新 episode 起点重新对齐，场景事件时钟归零 | odom 位姿跳变：只允许无 SLAM/AMCL/Nav2 的 sim_only 模式；episode+1，状态经 RESETTING→STARTING |
| 完整复位（其他模式） | 关闭整套 launch 后重启，新进程 /clock 从 0 起 | 新进程 | 新进程 | 新进程 | 新 ROS 图、新 run_id；无任何跨 run 缓存 |
| 检测到 /clock 回跳 | 设计上不发生 | — | — | — | run 判无效（acceptance §6），排查原因，不可计入成功 |
| 跌倒/故障 | 停止推进（非有限时）或按 FALLEN 规则停止 | 清空且拒收 | 不再推理 | 停止 | 停发；run 失败 |

## 8. 索引映射契约（W04 实现）

- t1_sim 启动时按名字解析：`RobotCfg.joint_names` 每个名字 → joint id → `jnt_qposadr`、`jnt_dofadr` → actuator id（`actuator_trnid[:,0]` 指向该 joint）。free joint `world_joint` 必须属于 `trunk` 且 qposadr=0、dofadr=0。
- W02 探针已证实原始模型满足：hinge qposadr=7..29、dofadr=6..28、actuator 0..22 与 `joint_names` 同序、gear=1。上游 `MujocoController.__init__`（qpos 全长拼接、ghost）与切片代码要求 **nq=30、nv=29、nu=23**。
- 场景只能增加：静态 geom、mocap body（动态障碍）、site、灯光、相机、材质。启动时断言 nq/nv/nu 不变且按名解析结果等于原切片；否则 `FAULT model_incompatible` 并拒绝启动，不静默运行。
- 需要带自由度的障碍或额外执行器时：先 ADR，改写构造与全部索引（system.md §3 中 W04 的覆盖点），并重验 W04/W05。
- `/joint_states.name` = `RobotCfg.joint_names`（23 个，真实名字）；`position=qpos[qposadr]`（rad）、`velocity=qvel[dofadr]`（rad/s）、`effort=qfrc_actuator[dofadr]`（N·m，已裁剪后的实际执行器力矩）。不包含 free joint、mocap 或场景物体。

## 9. /odom 计算（W06 实现，W02 给定语义）

记 `p=qpos[0:3]`、`q=qpos[3:7]`（wxyz）、`v_w=qvel[0:3]`（**世界系**线速度，trunk 原点）、`ω_b=qvel[3:6]`（**机体系**角速度），探针已证实。ψ 为 trunk 偏航。
- pose：position=(p.x, p.y, 0)，orientation=只含 ψ 的四元数（ROS xyzw 顺序，显式转换并归一化）。
- twist（child = base_footprint）：`linear.xy = R_z(ψ)ᵀ·v_w.xy`，`linear.z=0`；`angular.z = (R(q)·ω_b).z`（世界竖直分量），x/y 分量 0。base_footprint 是 trunk 原点的投影，故其速度就是 v_w 的水平分量，无需额外叉乘项。W06 以积分一致性（≤0.02 m 或给出依据）和旋转 90° 前进实验验收。
- 协方差：对角 1e-6，文档与 diagnostics 标明“仿真真值理想里程计，协方差为名义值，不是估计误差”。
- 不得把上游 `root_lin_vel_b`（实为世界系）直接当作机体系速度发布。

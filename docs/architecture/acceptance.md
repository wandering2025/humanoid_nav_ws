# 验收契约（acceptance v1）

状态：W02 首版（2026-10-13）。读者：实现与审查 W03–W15 的 agent。数值未测处标“待Wxx确定”，在对应步骤以证据冻结后填入本文件并记录 ADR；**已冻结的判据不能在看到结果后放宽**。接口语义见[interfaces.md](interfaces.md)，架构见[system.md](system.md)。

## 1. 判定词与证据

- `PASS`：判据满足，且有命令、cwd、环境、退出码、起止时间（sim 与墙钟）、被测完整 SHA、证据路径。
- `FAIL`：已运行且不满足。`NOT_RUN`：未运行（不能写成 PASS）。`BLOCKED`：缺少前置或输入，写明缺什么。`INVALID`：运行本身不符合协议（如 /clock 回跳、dirty 代码跑正式评测、参数与 manifest 不符），既不算成功也不从分母删除，需重跑并保留原记录。
- 启动进程、构建成功、窗口打开均不等于 PASS。GUI 判据需截图/录屏或用户亲眼确认。

## 2. 门槛

| 门槛 | 时间点 | 判据 | 失败处理 |
| --- | --- | --- | --- |
| G1 | W05 后 | 程序控制稳定行走、转弯、停车；`command_map.yaml` `calibrated:true` 且包络有证据 | 暂停 W06 及之后，留在 W05 查因；无可行包络则 W05 BLOCKED |
| G2 | W08 后 | 移动中时间/TF/odom/LiDAR 一致：墙面重合、无自扫描伪障碍、旋转不整幅错位 | 不进入 W09 |
| G3 | W11 后 | 静态 5 例预定义导航全部到达（§3 判据），无环境碰撞/跌倒 | 不进入 W12 |
| G4 | W12 后 | 横穿、持续阻塞需绕行、突然接近三类均可运行；至少一例真实绕行；不得以永久停车代替导航 | 不进入 W13 |
| G5 | W15 后 | 40 次正式评测证据、启动复现、技术说明、演示齐备，且证据与提交一致 | 不发布 |

## 3. 数值判据

| 项 | 初值 | 状态 | 冻结者 | 说明 |
| --- | --- | --- | --- | --- |
| 到达位置误差 | ≤0.30 m | W05：可达（低速停距 ≤0.14 m）；待W11冻结 | W05 提议，W11 冻结 | base_footprint 真值（换算到 map）与目标的平面距离 |
| 到达偏航误差 | ≤20° | W05：未证明可达（原地停车偏航回弹至 26.1°）；W11 须闭环实测，不可达则正式评测前 ADR 修订 | 同上 | — |
| 终点稳定 | 连续 2.0 s 仿真时间保持在容差内且静止 | W05 已定：0.5 s 窗口平面速度 <0.05 m/s 且偏航率 <0.05 rad/s（瞬时 \|v_meas\| 参考 0.05 m/s） | W05 | 站立最大 0.0088 m/s，步态最小 0.189 m/s（W05 locomotion-envelope §1） |
| 单例时间上限 | 120 s 仿真时间 | W05：维持 120 s，前提最长路径 ≤25 m；否则按 T ≥ L/0.27+25 s 预先修订（ADR） | W05 | 必须在正式评测前修订，不能事后调整 |
| 运动包络（vx 范围、wz 范围、组合、倒退/原地转/横移） | vx [0.315,0.75]；行走 \|wz\|≤0.45；原地 \|wz\| [0.56,0.85]；倒退/横移禁用 | W05 已定（ADR-010） | W05 | `command_map.yaml` 与 W05 证据 |
| 零命令停距/停时、起步延迟 | 0.75 m/s 停距 ≤0.353 m；行走停时 ≤1.82 s、原地 ≤3.12 s（停车偏航回弹至 26.1°）；起步 0.05 m ≤0.58 s、t90 ≤1.31 s | W05 已定（最坏值，含步态相位扫描） | W05 | 分速度表见 locomotion-envelope §4 |
| 命令超时 `cmd_timeout_sim`/`cmd_timeout_wall` | 0.5 s / 0.5 s | 待W10冻结 | W10 | 双条件（interfaces §6.3） |
| 停止区/减速区 | 无固定半径 | 待W10推导 | W10 | `d_stop ≥ v_max·(t_cmd + t_scan + t_compute) + d_brake_worst(W05) + 机体扫掠余量 + 裕量`，写出每项实测输入 |
| 最小净距目标 | 无 | 待W10推导 | W10 | 由上式与包络导出；见 §7 三种净距定义 |
| 成功率 | 静态 10/10；横穿/阻塞/突然接近各 ≥9/10 | 已定（总规划 §10） | — | 允许的一次失败须如实记为失败 |
| 环境碰撞、跌倒 | 正式案例 0 | 已定 | — | §7 定义 |
| RTF | headless 中位数 ≥1.0；GUI ≥0.7 | 目标值，未达标如实标记 | W13 测量 | 不得改 /clock 伪装实时 |
| W04 站立 | headless 60 s 仿真无 NaN/跌倒；同配置重启 ≥3 次 | 已定（W04） | — | — |
| W04 与上游对照 | 原始场景短运动与官方基线量级差 ≤10%，超出先解释时钟/初值 | 已定（W04） | — | — |
| W06 odom 积分一致 | 短试验位置差 ≤0.02 m（不适用时说明采样依据） | 已定（W06） | — | — |
| W08 测距 | 理想模式中央射线误差 ≤0.02 m；扫描计算 P95 < 扫描周期一半 | 已定（W08） | — | — |

## 4. 运行 ID、目录与 manifest

- `run_id = <UTC YYYYmmddTHHMMSSZ>_<suite>_<case>_s<seed>`，例：`20261015T093000Z_static_S03_s1002`。开发试验 suite 用 `dev`。sim_only 原地复位产生的 episode 记为 `<run_id>#e<n>`。
- 目录：`$WS/run_data/<run_id>/`（忽略）。轻量摘要复制到 `docs/evidence/Wxx/` 或 `docs/results/` 并追踪。批量中断后按 run_id 续跑，已有目录不覆盖。
- `manifest.json` 必填：`run_id`、`created_utc`、`code_sha`、`dirty`（及文件列表）、`upstream_sha.{booster_deploy,booster_assets}`、`checkpoint_sha256`、`mjcf_sha256`、`scene_file`+`scene_sha256`（组合后展开 MJCF）、`map_yaml`+`map_sha256`+`T_world_map`、参数文件 SHA256（nav2、slam、amcl、velocity_smoother、collision_monitor、command_map）、`mode`、`suite`、`case`、`seed`、`sensor_mode`（`compensated|ideal_stabilized`）、`odom_mode`（`ideal_truth`）、`initial_pose`、`goal`、`time_limit_s`、`tolerances`、`target_rtf`、`headless`、`policy_dt/decimation/physics_dt`、主机与版本（CPU、Python、ROS、MuJoCo、PyTorch）、启动命令行、退出码、`sim_time_end`、`wall_duration_s`。
- 正式评测（W13）要求 `dirty=false`；否则 INVALID。

## 5. CSV 字段

`sim_trace.csv`（t1_sim，50 Hz，每策略步一行）：
`sim_time, wall_mono, episode, state, cmd_status, cmd_source, v_cmd_in_x, v_cmd_in_y, v_cmd_in_wz, v_guard_x, v_guard_y, v_guard_wz, u_pol_x, u_pol_y, u_pol_wz, v_meas_x, v_meas_y, v_meas_wz, x_w, y_w, z_w, yaw_w, roll, pitch, proj_grav_z, action_norm, step_compute_ms, scan_due, scan_compute_ms, rtf_10s, nonfinite`
（`v_cmd_in`=收到的 /cmd_vel，`v_guard`=末端保护后的物理量，`u_pol`=策略输入，`v_meas`=实测；导航输出 `/cmd_vel_nav` 由 evaluator/rosbag 记录并按 sim time 对齐，用于 W11 的四层诊断。）

`contact_events.csv`（t1_sim，物理子步采样、去重）：
`episode, t_start, t_end, robot_geom, robot_body, other_geom, other_body, other_class, max_penetration_m, n_substeps`（`other_class ∈ {static_env, obstacle, ground_nonfoot}`）

`results.csv`（t1_evaluator，每 run 一行）：
`run_id, suite, case, seed, code_sha, dirty, outcome, nav2_result, arrived_truth, pos_err_m, yaw_err_deg, settle_s, time_to_goal_s, path_len_m, clear_body_env_min_m, clear_footprint_min_m, scan_min_range_m, n_env_collisions, fell, n_safety_stops, n_replans, n_recoveries, rtf_median, sim_time_s, wall_time_s, notes`
`outcome ∈ {SUCCESS, FAIL_COLLISION, FAIL_FALL, FAIL_TIMEOUT, FAIL_SAFETY_STOP_UNRECOVERED, FAIL_LOCALIZATION, FAIL_PLANNING, FAIL_STARTUP, INVALID}`。

`fault_tests.csv`（W10）：
`test_id, fault, injection_method, t_inject_sim, t_detect_sim, t_zero_cmd_sim, v_meas_at_inject, stop_distance_m, final_state, expected, verdict, evidence`

列可追加，不可改名或改义；改义须提升文件头 `# schema=<n>` 并 ADR。

## 6. 测试集与种子（预注册）

| suite | case | 次数 | 种子 | 变化量（在 W12/W13 测试集文件中逐行列出） |
| --- | --- | --- | --- | --- |
| static | S01–S10 | 10 | 1000–1009 | 起点/目标组合（含 W11 的 5 例），至少一例需绕静态障碍 |
| crossing | C01–C10 | 10 | 2000–2009 | 障碍速度、出现时间/触发位置 |
| blocking | B01–B10 | 10 | 3000–3009 | 阻塞位置、停留时长，均存在可行绕行路线 |
| sudden | U01–U10 | 10 | 4000–4009 | 进入保护区的起始间距（须允许物理制动）、离开条件 |

- 测试集文件（位置由 W12/W13 定，建议 `src/t1_bringup/config/test_suite_v1.yaml`）必须在第一次正式运行**之前**提交，记录其 SHA；之后任何修改成为新版本，并完整重跑该版本。不得按结果删行或重排。
- 种子决定障碍时序与（若启用）传感器噪声；同种子同 SHA 的 run 应可复现到事件级（RTF 不同不影响仿真时间行为）。
- 用户因时限缩减样本时，结果标“缩减评测”，保留原 40 次目标与覆盖缺口。
- 每个 run 单独完整 launch（system.md §7），不得继承上一 run 的地图变化、策略历史、TF 缓存。

## 7. 指标定义

- **到达成功**：Nav2 NavigateToPose 结果 SUCCEEDED **且**真值满足 §3 位置/偏航/稳定判据，且在时间上限内。只满足其一记为失败并在 notes 说明。
- **环境碰撞**：任一机器人碰撞几何（`body_rootid==trunk`、碰撞组）与任一非地面环境或障碍几何的接触（`dist ≤ 0`）。机器人自碰撞不计；脚部几何与地面接触不计；非脚部几何接触地面记 `ground_nonfoot`，计为失败（通常伴随跌倒）。脚部几何集合由 W07 依据实际 body 名确认（预期为左右 `*_ankle_roll_link` 下的几何）。事件去重：同一几何对连续出现视为一个事件，间断 ≥2 个策略步（0.04 s）才算新事件。
- **跌倒**：上游判据投影重力 z > −0.5（倾角 >60°），或 trunk 高度低于阈值（W05 提议 0.50 m：1404 次标定最低 0.637 m；W10 冻结）。
- **净距（三种，不可混称）**：`clear_body_env_min`=机器人碰撞几何与非地面环境/障碍几何的最小三维距离（仿真内计算，首选评测量）；`clear_footprint_min`=保守 footprint 多边形到障碍几何的平面最小距离；`scan_min_range`=/scan 有效最小量程。
- **安全停车**：collision_monitor 进入 stop 动作或 t1_sim `cmd_status ∈ {TIMEOUT, INVALID, BLOCKED_BY_STATE}` 导致 u_pol=0 的事件；记次数与恢复时间。安全停车后未恢复到达 → `FAIL_SAFETY_STOP_UNRECOVERED`。
- **RTF**：10 s 墙钟窗口内 Δsim/Δwall；报告中位数与 P5，headless 与 GUI 分开，附硬件与进程配置。

## 8. ROS 图静态检查（每次集成步骤 W06、W08–W13 执行并留证据）

| 检查 | 命令（在对应模式运行中） | 期望 |
| --- | --- | --- |
| /clock 唯一 | `ros2 topic info /clock -v` | 发布者恰为 1 个，节点 t1_sim |
| /cmd_vel 唯一（W10 起） | `ros2 topic info /cmd_vel -v` | 发布者只有 collision_monitor；订阅者 t1_sim |
| TF 唯一 authority | `ros2 run tf2_ros tf2_monitor`（足够时长） | 每帧单一 authority；map→odom 为 slam_toolbox（mapping）或 amcl（navigation）；无 t1_sim 的 map 帧 |
| 真值隔离 | `ros2 topic info /t1/eval/ground_truth -v`；`/t1/viz/*` 同查 | 订阅者 ⊆ {t1_evaluator, 记录器, rviz（仅 viz）} |
| 调试入口关闭 | `ros2 topic info /t1/debug/policy_cmd -v`（navigation/evaluation） | 无订阅者 |
| 无时间戳命令 | `ros2 param get <nav2 节点> enable_stamped_cmd_vel` | 全部 false；/cmd_vel* 类型均为 geometry_msgs/msg/Twist |
| 仿真时间 | `ros2 param get <节点> use_sim_time` | 除 t1_sim 外全部 true |
| 机器人 frame | 检查 costmap/CM/AMCL/SLAM 参数 | 全部 base_footprint |

## 9. 故障实验矩阵（W04 做 F01–F05、F13；W10 做全部并写 fault_tests.csv）

| ID | 故障 | 期望行为 |
| --- | --- | --- |
| F01 | 停止发布 /cmd_vel | 双超时后 u_pol=0，继续站立；`cmd_status=TIMEOUT` |
| F02 | /cmd_vel 含 NaN/Inf | 本步 u_pol=0，`INVALID`，不锁死后续合法命令 |
| F03 | 超限命令 | 裁剪到包络；未验证自由度置零 |
| F04 | 暂停后恢复（暂停前有非零命令） | 恢复后无新命令不运动 |
| F05 | Ctrl+C / SIGTERM | 有序退出，退出码记录（期望 0），无遗留进程；GUI 退出段错误单独记录不得吞掉 |
| F06 | 停止 /scan | collision_monitor `source_timeout` 后停止；t1_sim 扫描健康门置零 |
| F07 | 停止 TF/odom | collision_monitor 变换失败→停止；不沿旧速度继续 |
| F08 | collision_monitor 进程退出 | /cmd_vel 超时→零；不允许运动 |
| F09 | 导航取消/更换目标 | 旧目标命令在超时内归零，不继续执行 |
| F10 | 静态障碍逼近停止区 | 记录触发时刻、末速、实际停距 ≤ 包络+裕量 |
| F11 | 减速命令落入死区 | 不被末端放大（`|M⁻¹(u_pol)| ≤ |v|`） |
| F12 | 倾角/高度超限（受控注入） | FALLEN，run 终止为失败，不自动恢复 |
| F13 | 策略异常/非有限动作（测试钩子注入） | FAULT，u_pol=0，停止推进（非有限时） |

## 10. 证据位置

| 类别 | 位置 | 追踪 |
| --- | --- | --- |
| 每步检查点证据 | `docs/evidence/Wxx/cN-*.{txt,json,csv,md,png}` | 是（每文件 <1 MB） |
| 大日志、rosbag/mcap、视频 | `docs/evidence/Wxx/raw/`、`run_data/` | 否（.gitignore 已覆盖） |
| run 原始数据 | `run_data/<run_id>/` | 否 |
| 评测汇总 | `docs/results/`（W13：`evaluation.md`、`results.csv`、失败表） | 是 |
| 运动包络 | `docs/evidence/W05/locomotion-envelope.md` + `src/t1_mujoco/config/command_map.yaml` | 是 |

## 11. W02 自身验收

| 验收项 | 结果见 |
| --- | --- |
| 后续每步能找到输入、输出、停止条件、证据定义 | system.md §9、本文件 §2–§10 |
| 无“后面再随便选”的关键接口 | 余下选择均为有界二选一或待测数值，并指定所有者：/scan 生成方式（W08 在 D5 两种方式中选，frame 固定 `laser_stabilized`）、/clock 提频（W06/W11，需 ADR）、/imu 是否发布（W06，默认不发）、数值（W05/W10） |
| 无双发布、环路、重复真相、隐含依赖 | system.md §10 |
| 数据流与控制周期 | system.md §5–§6 |
| ≥3 个备选方案及未采用原因 | system.md §11 |

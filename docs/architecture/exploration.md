# W16：仅知 B 相对 A 的在线探索

本实现是位置目标导向的 frontier 探索，独立入口 `scripts/run_exploration_demo.sh`。默认同时打开MuJoCo与RViz，执行未知环境探索与真实动态障碍避让；运行结果见 [W16 记录](../plans/t1_mujoco_navigation/records/W16.md)。原 guided 演示保留原有接口和证据。

## 输入与坐标

任务文件 `src/t1_navigation/config/exploration_task.yaml` 只接受 `schema: 1`、`frame: start`、`goal: {x, y}`。不接受路线、房间范围、终点朝向、固定进近方向，也不读取场景做目标可达性预检。目标可以在地图外或墙中，这些情况由运行时观测处理。

A 定义为探索开始、机器人就绪站立时的 base_footprint 坐标系；+x 为该时刻身体朝向，+y 为左侧。独立探索进程通过 `odom→base_footprint` 锁存 A，把 B 的相对坐标变换到 odom；每次决策通过当前 `map→odom` 换算 B。A 的位置/朝向和 B 的解析结果写入 result.json。就绪前步态可能产生少量偏航，不能把相对坐标机械当作 world/map 坐标。

这依赖当前已披露的理想 odom。没有新增 TF 发布者，也没有用模拟器直接发布 map→odom。真实有漂移的里程计需要进一步用 SLAM 图中的起点锚定，当前不作该能力声明。

## 决策与信息隔离

`scripts/exploration/controller.py` 是独立 `t1_explorer` 进程。唯一任务输入为运行目录中不可变的 task.yaml 副本；它不读取 world YAML、地图文件、旧航点、presentation/goal.yaml、覆盖区域或评测结果。

| 接口 | 用途 |
| --- | --- |
| `/map` | SLAM 实际观测与审计快照 |
| `/global_costmap/costmap` | SLAM + 实时 scan 的已知自由区、障碍膨胀、frontier 与可达性 |
| `/tf`、`/tf_static`、`/clock` | 位姿、坐标换算、仿真时限 |
| `/t1/safety/state` | 准入和故障状态；意外安全锁存后退出 |
| `ComputePathToPose`、`NavigateToPose` | Nav2 路线验证与执行 |
| `/t1/safety/arm`、`/t1/safety/stop` | 开始时显式 arm，结束/失败时锁存停车 |
| `/t1/exploration/status` | 决策、失效、到达等观测事件 |

评测进程可订阅 truth，只记录与验收。它不提供探索目标；模拟器场景路径只传给 launch。实际 ROS graph 与 RMW TF GID 被独立记录，最终速度仍只由 Collision Monitor 发布。

纯算法 `t1_bringup/exploration.py` 的输入仅为栅格、分辨率、origin、当前位姿、B、历史访问/失败点。初始 SLAM 栅格很稀疏，因此使用 Nav2 真实在线全局 costmap，而不是把初始空洞填成自由区。costmap -1 始终未知且不可通行，99/100 为不可通行；0–98 是自由/膨胀代价，不混用占据概率阈值。

从已知自由格与未知格的交界生成候选，仅选择机器人所在可达连通区域内的已知点。候选还避开已膨胀障碍边缘0.20m；保留原2.05m导航圆、1.65/1.95m Collision Monitor 区和速度安全链。该检查针对路径中心格，完整机器人避障由 Nav2 膨胀、实时雷达与原安全链共同执行；不声称整个未知边界已被观测。

评分使用可达行程代价、到 B 的启发距离、沿观测射线可看到的 frontier 角度以及访问惩罚。射线在已知障碍处终止，墙后未知不被当作已看见；B 距离只是偏好，允许先远离 B 绕墙。失败点暂避，重复访问受惩罚。使用原 NavFn 验证候选路径，逐线段核对所有中心格已知可达。

执行中每3仿真秒检查新地图：当前探索目标失效时重新选点；B 出现已观测安全路线时直接抢占去 B。探索点0.65m通过半径不算最终到达，位置误差≤0.30m且物理站稳≥2秒才算任务完成。终点朝向由在线进近路径决定，不是环境预置。全程 SLAM 保持运行，无整房覆盖比例条件，不需要先完成建筑地图。

## 有界失败与证据

默认总预算600仿真秒、最多40次决策、单目标120仿真秒；ROS调用和墙钟有独立上限。无可达 frontier 时取消当前动作，最多等待20仿真秒的新观测，避免把临时阻塞立即判成永久死路。等待会明确锁存停车；只有自己发起的等待且gate无健康故障时，才在新路径成立后显式arm，仍由原gate检查新鲜零命令。其他取消/故障不会自动恢复。等待仍计入总预算；耗尽、超时、TF/clock或安全故障时锁存停车并退出，保留失败结果。失败只说明本次未建立可行路线，不证明任意未知环境中 B 永远不可达。

`explorer/decisions.jsonl` 保存位姿、候选分数、实际 NavFn 路径、动作ID和重选原因；`map-*.npz` 保存每次选择时的 costmap 与 SLAM 栅格；`result.json` 保存 A/B 换算及停稳检查。独立评测再用物理 trace 核验位置、碰撞、跌倒、停车、组件退出、进程与端口回收。原始数据留在 ignored run_data，轻量报告在 docs/evidence/W16。

限制：未验证任意动态场景、漂移里程计、真机、狭窄通道或普遍完备性；依然是有限束 LiDAR、理想姿态补偿和受限 T1 步态。少量正/负例不能替代可靠性评测。原 W13 的 FAILED 结论不变。

算法依据：[frontier 原论文](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf)。在线地图接入依据：[Nav2 Jazzy SLAM 教程](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/)。这里的目标偏好、T1 安全约束及审计为本项目实现。

## W16 动态障碍扩展（ADR-023）

默认入口使用 `exploration_events.yaml` 的 EXP01；`--static` 完全不实例化动态实体，供对照和其他静态布局使用。3.0×0.6×1.5m方体由已有 physics owner 在每2ms物理步前移动，0.6m/s进入通道、停12s、再退回；实际 mocap 碰撞体参加 mj_step 接触检测和 LiDAR 射线，不只是视觉标记。轨迹和位置触发条件只在模拟器内部；探索和感知节点不接收它们。该物体是不可推动的运动学障碍，不代表物理驱动的行人。

新增独立 `t1_mapping_scan_filter`，仅订阅原始 `/scan`、`/tf`、`/tf_static`、`/clock`，根据每帧原时间戳查询 odom→laser_stabilized。输入历史初始为空，不读 world/map 文件或 truth。0.1m稀疏格记录有限回波之前、距离端点至少0.35m的实际射线自由证据；每格每帧最多一次，至少3帧。+inf保留，但不用于长期自由先验，避免角格投影在墙角产生过强的自由证据。

进入此前已观测自由格的回波判为瞬态，并沿间距<0.3m的相邻回波传播。至少3帧的稳定表面及其0.2m格邻域阻断这种传播，防止墙角单点误判删除整面墙；稳定先验不向此前自由区扩张，防止慢移物体把自己不断扩充为静态结构。暂时停住的已检测物体继续排除。

过滤仅将对应量程置NaN，保留消息时间戳、frame、角度及其他量程；不会输出穿过物体的假射线。已安装Karto头文件明确在scan matching与occupancy rasterization忽略NaN（本地 `Karto.h`，源见[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)）。SLAM只订阅 `/scan/mapping`，仍是唯一/map与map→odom发布者，始终处理新观测；Nav2两层costmap与Collision Monitor继续订阅完整的原始 `/scan`。原SLAM参数只在运行目录复制并改变scan_topic，保存实际哈希；旧入口/参数不改。

滤波器不能可靠识别最初在未知区域静止的物体，也不能修复任意已进入历史pose graph的动态回波；当前位置分类依赖本项目理想odom和有限LiDAR。这是经过具体场景验证的观测变化过滤，不宣称通用动态SLAM或运动预测。

评测独立保存实际障碍位姿、雷达命中、costmap标记、命令和物理轨迹，检查≥2s站稳等待、离开后恢复≥1m、到B≤0.30m与安全清理。`scan-filter.jsonl`记录每帧被排除的索引/odom端点、处理耗时和历史格数；离线 `dynamic_report.py` 按时间对齐障碍命中与过滤索引，并从真实最终SLAM图检查阻塞位置是否留下占据残影。原始观测留ignored目录，轻量报告进入W16证据。

## W16 图形入口（ADR-024）

默认 `./scripts/run_exploration_demo.sh` 同时打开 MuJoCo 和 RViz；`--headless` 保留CPU验证，`--no-rviz`只显示MuJoCo，`--hold`到达站稳后保持窗口直到Ctrl+C。沿用既有D3D12/NVIDIA选择、GPU/宿主日志监控、退出join与精确PID回收；不新增渲染后端。

探索状态单向发布到只读显示端。A/B采用explorer的start事件中的anchor_odom和goal_odom；MuJoCo仅改独立view_model/user_scn，RViz在odom帧发布标记并由真实map→odom换算。探索开始前不展示假定的A/B。RViz显示实际SLAM格、原始雷达、实际Nav2路径、探索器当前目标和实际TF轨迹，不计算旧guided路线概览。模拟器接收状态仅为显示，无真值/场景数据返回给探索器，不改物理状态、TF或速度ownership。

运行保存ready/dynamic/arrived实际窗口截图、GPU监控、显示坐标与导航结果一致性检查；到达后保留窗口8秒再按原流程停车清理（--hold除外）。本轮不改presentation/，待用户测试反馈后再加入新命令。

图形负载下的建图实时性：scan_history以整数tuple集合去重代替逐帧结构数组排序，保留原格子和每帧确认计数；120帧包含转向、移动障碍与非有限量程的差分测试对旧实现完全一致。建图分支订阅/待处理队列深度1，允许丢弃被更新帧替代的旧scan，始终按实际scan原时间戳查TF并发布；不重写时间戳，不改SLAM参数/导航或安全阈值。记录output_age_s用于验证负载时延。原始scan发布及Nav2/CM订阅均未改变。

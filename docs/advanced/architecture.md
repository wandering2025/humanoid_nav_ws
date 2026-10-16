# Advanced demo：连续建图与可见动态干扰

本页描述优化分支当前实现；冻结的旧版架构可从 `demo-v0.2.0` 查看。当前验收状态见 [optimization-review.md](optimization-review.md)。

## 运动与地图流程

物理 T1 policy/PD/mj_step、LiDAR射线、导航适配器、安全门、平滑器和 Collision Monitor 均复用原实现。没有编辑机器人 base pose，没有把场景几何栅格化后提供给规划器。

1. 从空白 SLAM 会话开始，Nav2在已观测自由空间自主通过观测航点；`allow_unknown:false`。
2. 航点为(3.5,0)→(3.5,3.5)→(3.5,7.8)→(7.5,8)→(11.5,6.5)，必要时到(13.4,6.5)。0.65m是观测点通过半径，不是最终到达容差。下一目标直接 preempt；不 cancel/re-arm，不要求逐点精确停稳。
3. 第5点起等待原1Hz地图/costmap更新，再由 NavFn ComputePathToPose检查A→B。同时要求东区主通道观测区域[9,14]×[−2,10]已知比例≥95%，才结束观测路线；不再返回A。该区域不包含x≥14.4的停放障碍遮挡区；完整东区[9,16]的覆盖率单独报告。区域仅是覆盖目标，不是占据地图先验。青色概览是这个真实规划结果，只用于显示，不驱动机器人返回A。
4. 调用SLAM已安装版本的暂停接口，读参数确认状态。保存真实`/map`，记录地图几何/数据指纹。保留最后的map→odom修正，障碍处理阶段显式依赖理想里程计。实时LiDAR仍进入costmaps及安全系统。
5. 启用场景事件；导航到(12,2.5)的通过点，保留工具启动期间的行进余量，防止越过1.5m直线接近位置。动态物体的truth话题只由评测节点订阅；没有给Nav2未来运动或真值位置。
6. 原工具完成B=(12,0)及可行的直线接近，严格验证停稳；核对导航阶段地图指纹未变，再恢复SLAM、保存最终地图、安全停车和有界清理。B距南墙4m，保留充分的停车及恢复空间。

持续建图会把较宽移动障碍写入累积静态图，使撤离后仍无法规划；失败01保留。分阶段地图冻结避免此问题，但不提供该阶段的LiDAR定位修正。依据安装版本[暂停接口与TF发布](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp)及[异步扫描回调](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_async.cpp)实现；没有修改SLAM依赖源码或新建TF发布者。

## 可见干扰与规划

东区3×0.6×1.5m的mocap障碍从(15.9,3)以0.6m/s进入(12.5,3)，停8s再撤离。事件必须在建图阶段之后显式启用，且x≥10、前进指令>0.1才触发。几何/时序制造真实通路阻塞；车体不能被推动，不代表人体交互模型。

高级模式用已安装的Regulated Pure Pursuit替代MPPI作为RotationShim的主跟踪器；NavFn与5秒重规划逻辑不变。单独的advanced_controller.yaml只作用于高级模式，默认导航仍使用原MPPI。0.40m/s目标速度不超过原0.45m/s，前视0.8m，保留碰撞预测、禁用倒走；接近减速适配现有步行下限，未修改适配器。package.xml仅声明已安装的RPP运行依赖，没有安装或升级环境。实际插件在启动时查询确认。选择依据是实测MPPI在最低步行速度/航向误差下错过目标并绕圈；RPP提供直接几何路径跟踪。[已安装接口对应的上游实现](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)。

高级专用BT临时阻塞恢复为Wait4秒、最多6次，不使用spin/backup。观测目标各120sim s、最终目标200sim s，失败不会被改写为成功。预抢占偶发旧FollowPath提前返回SUCCEEDED时，仍用实际地图位姿检查；最多重新发送2次，ABORTED/CANCELED立即失败。

RViz蓝线复制实际`/plan`的帧、时间戳和poses到持久化显示话题；新规划到达就更新，不伪造理想轨迹。规划失败等待时显示最后一次成功路径及其年龄，不能把旧蓝线视为当前保证可执行的路径。青线是A-B地图概览、紫线是TF测得轨迹、橙色是当前目标；显示都不参与控制。

## 保持与收紧的标准

规划半径2.05m、停止区1.65m、减速区1.95m、膨胀2.6m、原速度/停稳/跌倒标准未降低。原navigation.yaml、安全文件、适配器、runner、run_demo.sh与稳定tag字节一致。目标工具只增加高级入口显式启用的`--require-clock`，在收到有效仿真时钟前不发目标；去掉该选项及检查共3行后与稳定源码完全相同。默认行为和原工作区文件未改。实验中的0.12m内部容差已撤回；Nav2与最终工具均保持稳定版的0.20m/0.20rad，停止速度标准未变。启动查询确认实际参数和插件。

动态验收要求≥3帧扫描命中、≥2帧costmap标记、≥2s实际静止且最终命令为零、距B>1m、障碍结束后行进≥1m并到达。仅有路径偏移不再放行。SLAM有序deactivate/cleanup、按登记PID关闭、端口释放和四项零碰撞/跌倒仍是硬条件。

## 限制

预定义观测航点与场景坐标，不是真正frontier exploration；理想真值里程计、真值姿态LiDAR补偿、有限9×360射线/10Hz/1cm噪声仍在使用。动态障碍处理阶段暂停SLAM修正，不能称为持续LiDAR定位。静态图包含建图时停放的可移动物体，它后方仍有未知区和东墙局部缺口；地图表达主要结构，不是建筑每面墙已完整扫描。动态层的反应并非轨迹预测。步态漂移与终端有界重试仍可能出现；两次通过不是可靠性认证。采样机体间距不是连续时间下界。W13仍FAILED36/40。当前优化图形与录像验收通过，见验收记录；原GPU锁已保留归档，没有更换驱动、环境或录像宿主。


## 图形恢复与显示消息归属

高级入口启动准入为连续5秒GPU温度≤60°C、利用率≤60%，默认监控调用仍用40%。这一高级专用调整基于真实桌面短时负载；运行中75°C、70%显存、相对基线增长768MiB、95%利用率持续5秒，以及宿主/图形错误监控均原值。此前显存告警锁逐字节归档，未删除失败证据或修改驱动。最小图形、完整排练和独立录制均完成。

NavFn ComputePathToPose的概览结果与其/plan发布存在异步时序；仅检查“正在算概览”标志会漏掉迟到消息。现在保存概览的坐标系和完整路径几何，排除迟到概览，并只对实际转发的执行路径更新年龄。蓝线消息直接原样发布，青线概览不用于发送机器人目标。完整运行核验23条几何/坐标系匹配、概览不混入蓝线，11条序列化字节完全一致；CDR填充字节的不稳定性与初次严格审计失败均如实保留。


## Presentation B-point interface

`presentation/goal.yaml` is the operator source for each launch. The advanced orchestrator always rereads it, validates finite x/y within[11,12]×[-.5,.5]m in map, fixes heading south and derives the handover(x,y+2.5). The entire vertical segment is checked against walls/static boxes/parked cart for≥2.4m clearance; this is operator admission only, not an injected SLAM map. Existing planning/safety/controller thresholds remain unchanged.

`presentation-goal.json` stores exact UTF-8 source text/hash and resolved targets. The orchestrator passes its snapshot path to its children via `T1_ADVANCED_GOAL_SNAPSHOT`; the advanced simulator verifies the snapshot and writes the same goal/provenance into its manifest. Navigation, real NavFn overview, RViz, render-only MuJoCo annotations and arrival metrics use this goal. Verification rejects a mixed simulator/navigation goal. Standalone advanced simulation reads the YAML when no internal snapshot is supplied. No non-advanced mode reads this file. The world YAML goal is nominal scene metadata; advanced_route.yaml no longer duplicates B or its handover.

The above fixed coordinates describe the default(12,0). Nonstraight-route validation is now relative to A-B distance(>1.4×) with the existing north-gallery passage check, so selecting another supported B does not trigger a hardcoded19m quality threshold. No robot safety threshold changed. Editing the file during a run affects only the next launch. See[operator rules](../../presentation/GOAL_CONFIGURATION.md) and[actual validation](../../presentation/VALIDATION.md).

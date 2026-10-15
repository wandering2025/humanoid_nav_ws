# W11R1 原始证据诊断（开发检查点）

起始 b23a92465b317ca781d3eb210051126c5786908e。原始文件不改；raw/formal-v1-baseline-hashes.json 保存1371个文件的SHA256。所有文件相对路径以仓库为根。

## C01

原run 20261014T222232Z_formal-v1_C01_s2000_3cddcf。launch日志41行发出Configuring map_server，44–55行实际加载地图成功，56行明确 `failed to send response to /map_server/change_state (timeout): client will not receive response`。之后没有AMCL配置激活；initialpose退出1；map→odom不存在；starter未发STARTUP，目标未发送。shutdown定位manager最终被SIGKILL。

直接原因：已执行转换的响应丢失后，Jazzy manager使用无界change_state等待，无状态协调；固定3s/8s延迟不能保证新建client的响应端已发现。底层FastDDS/WSL具体丢包机制未证实，不能把猜测写成结论。使用[固定1.3.13官方源码](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_util/src/lifecycle_service_client.cpp)及本机头文件核对无超时调用链。

## C09

原run 20261014T223423Z_formal-v1_C09_s2008_70370c。日志195–224证明controller/local costmap/planner/global costmap/behavior/bt均active。227首目标请求；228 FollowPath ACK超时，229服务已收到clear请求但230–231 BT按20ms超时，两次目标在0.16sim_s内ABORTED0。240行controller在上层已中止后才收到并执行goal，距初始请求约0.27墙钟秒。不能说server不存在。

直接原因：图发现/active替代了RPC可用性确认；20ms确认预算低于实际请求执行延迟；BT ACK失败后丢goal handle造成迟到FollowPath继续执行。旧适配器只在终态瞬间清cache，随后无active goal的raw又可进入。目标工具忽略approach ABORTED并发送终目标。错误码0未被评测判成功，但失败链缺乏清理。

## B04/B06

B04原run 20261014T224241Z_formal-v1_B04_s3003_ff4d28，approach passed_pose=[7.706993,-0.281345,1.655849]，51.28sim；B06原run 20261014T224608Z_formal-v1_B06_s3005_baa600，passed_pose=[7.650173,-0.312482,1.601123]，53.00sim。0.35m半径判据均满足，但航向95/92度；位置抢占没有实现ADR-016所述沿目标朝向进近。轨迹显示原地转向后回弹与最小步态抬升使侧向误差继续增大。

B04在62.02s为(8.870,.539,.406rad)，64.02s(9.409,.765,.467)，随后停在(9.54,.79)。日志268起多次从(9.52,.77)规划回(9,0)失败208。东墙11.5，中心净距约1.96m小于规划2.05m圆，起点进入inscribed膨胀区，清代价地图不能清除静态墙；W10 slowdown仍正确作用。接触/跌倒0。

B06在62.02s(8.847,.307,.299)，64.02s(9.303,.422,.376)，随后原地转到约−1.2rad并站立；20s无进度触发两次恢复，23条plan，无倒退情况下不能回收端点误差，到120sim_s取消。终点(.589m,62.4°)。两个原run终目标均未SUCCEEDED，因此成功后settle/最多3次积分修正均未进入；既有纠偏对正在失败的approach无作用。

## 单假设实验计划与边界

H1：有界lifecycle转换+状态读回能覆盖C01已执行丢ACK，不重复转换；ROS服务假节点测试及C01冒烟验证。
H2：真实FollowPath无运动探测（空path预期INVALID_PATH103）+costmap响应、500ms有限BT预算及命令代次清除，覆盖C09迟到请求；用响应超时注入及C09冒烟。
H3：预目标位置+航向联合交接能阻止B04/B06的侧向进入；先以原B04物理单例检验，失败则保留并分析，不盲目重复。

W05 command_map、安全链所有W10文件、场景/地图/事件/案例/评测器未改。禁止增加倒退/横移/spin/backup恢复。现有3s/8s创建节拍暂保留作为负载安排，正确性由事件协议与期限保证。

## 开发发现 H4 / H5

H3位置+航向抢占仍在旧FollowPath成功边界触发0.04s新目标假成功；独立定位核验将其拒绝。H4改为预目标action完成后静止2s联合核验，最多3次纠偏共享于两阶段。B04 H4到达.118662m/.908898°，证明该单次路径能在冻结包络内收敛；不是可靠率证明。

H5严格关闭审计发现旧候选996f36a的C01/C09虽实际到达，但新的manager在SIGINT后析构active bond创建timer时context已无效，exit -6。B04主动中断/B06未执行，旧候选拒绝。修复为pre-shutdown清bond、取消pending RPC并用锁串行化控制状态；真实ROS测试覆盖形成bond后SIGINT退出0。以前开发run无遗留PID不等于正常退出，后续必须逐子进程审计。

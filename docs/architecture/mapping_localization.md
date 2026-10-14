# W09 建图和定位接口

W09提供两个独立模式：`mapping.launch.py` 与 `localization.launch.py`，默认静态场 `static_world`。两入口也可由 bringup/simulation 的 `mode` 参数选择。ADR-013说明新增localization模式的边界；navigation/evaluation仍拒绝启动，安全链/自主目标由W10/W11负责。

所有消费者use_sim_time=true；模拟器自身false，消息stamp来自data.time。SLAM/AMCL通过真实scan stamp查询`odom→base_footprint→laser_stabilized`。不会发布世界真值map→odom。mapping只含SLAM；localization按map_server→AMCL顺序configure/activate，生命周期管理器维持bond。两种模式均拒绝debug策略命令和原地reset，切换模式要关闭完整launch再重启。

配置来自本机Jazzy安装样例：SLAM Toolbox online async（Ceres、0.05m网格、1s更新、0.15m/rad运动门槛）；保留原局部/回环搜索窗口。AMCL likelihood_field、DifferentialMotionModel、180束、500–2000粒子。alpha=0.02适配本项目理想odom，sigma_hit=0.1m适配5cm网格及W08角量化；参数不是漂移鲁棒性结论。

定位启动后等待map_server和amcl均active，通过`ros2 run t1_bringup initial_pose X Y YAW`只发送一次map系近似位姿（yaw单位rad，位置std=0.3m、yaw std=0.3rad）；也可用RViz的2D Pose Estimate。初始化节点不订阅truth/odom，不发布TF，不做周期重置。需要真实策略转动/行走采集进一步扫描，静止时AMCL会受update_min_d/a门槛限制。

`static_world.yaml/.pgm`来自SLAM Toolbox `/map`，通过nav2_map_server保存；不是场景栅格化。`.meta.yaml`绑定map YAML/图像/场景/参数哈希、生成源码SHA、run ID、传感器模式及`T_world_map`。此变换通过建图起点map→odom反求，利用W06的odom=world水平真值投影；只供仿真日志和评测/A/B换算，不注入AMCL。启动localization时模拟器检查地图及场景哈希，记录参数与地图版本。修改场景或地图须重建匹配元数据。

地图覆盖A/B及南北通道；背向路线且未被LiDAR照到的物体背面可能未知，不伪造占据格。可选序列化pose graph留在ignored raw供继续建图调试；AMCL只需追踪的yaml/image。后续步骤可以直接复用该地图，不必重新建图。

建图手动检查路线在W09证据脚本中预先指定，用受控Twist重放，以odom判断转向/停止；无Nav2规划器。`vx=0.4m/s`、原地`|wz|=0.6rad/s`及行走转向均经过W05标定与W04双超时保护。不能称为自主到点或完整碰撞保护。

验收与限制见[W09证据](../evidence/W09/README.md)：理想真值里程计、真值姿态补偿、默认无噪声瞬时LiDAR。三次小偏差初始化证明此静态场定位链路可重复运行，不证明真实机器人或带漂移SLAM的鲁棒性。

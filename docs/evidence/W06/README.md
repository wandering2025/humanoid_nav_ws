# W06 ROS 状态桥接与 TF 时间验收

2026-10-14，cwd=/home/haoyang/humanoid_nav_ws。C1–C4 PASS，仅 W06。
实现提交：`7fca2597388986fcaddbf3689b1ca5f563ce1e4d`；C3 检查点/运动被测版本：`15046be93f645265bd362d98d799efbdd248f947`。
`source-audit.json` 验证 src/scripts 与实现提交完全一致，C2 单测源码哈希一致。
后续提交仅证据、使用文档、完成记录，完整 SHA 见 records/W06.md。

## 产物与验收

| 验收 | 结果 | 实测/证据 |
| --- | --- | --- |
| 坐标/API 所有权 | PASS | velocity-probe.json：倾斜+旋转+混合速度，XBODY 世界/机体速度语义 5/5；COM 速度确实不同 |
| 旋转约90°后前进 | PASS | motion-integration.json：通过实际策略原地转至112°再停车，4 s后航向93.383°；前进8.0 s，世界位移 (−0.255909,+2.543791) m，child vx均值+0.356446 m/s，vy+0.007846 m/s；没有用基座位姿编辑行走 |
| odom积分与位移 | PASS | motion：23.44 s 仿真、50 Hz，最大累计 XY 差0.006138 m（阈值0.02 m）；controls 0.001921 m，restart 0.000739 m。将child twist旋回world后按相邻0.02 s样本梯形积分；reset跳变不跨段积分 |
| odom与真值同源/stamp | PASS | motion中1172组odom/truth/joints/两条base TF的stamp完全相同；XY误差0；stamp 0.02–23.44 s；clock含启动0，共1173条 |
| TF唯一树及异常时间 | PASS | 两个dynamic publisher（t1_sim、RSP）、一个static publisher（RSP）；child唯一父、无环；无map/odom静态替代，无真值frame入树。motion 226次回溯0.1 s TF查询0失败；controls/reset/restart无持续外推错误 |
| 暂停/恢复/重置 | PASS | controls 21/21，restart 15/15；暂停1.1 s墙钟clock/odom冻结，u_pol=0；暂停期间发命令被丢弃；恢复无旧命令，新命令有效；reset通知episode+1、清命令/分频、clock单调；完整新run重启消费者/Buffer |
| JointState名字/数量/单位 | PASS | model-export.json：静态150步+运动250步（8 s sim），23个真实joint逐名对照qposadr/dofadr；position rad、velocity rad/s、effort N·m最大数值误差均0，非机器人DOF未混入 |
| 速度、姿态、线程 | PASS | model-export.json：fresh复制MjData上的mjOBJ_XBODY与truth局部速度/odom世界wz最大差<9e-16，四元数差<2e-15；odom与W05 v_meas差<3e-16；异线程capture拒绝。策略观测、权重、PD与环境未改 |
| RViz实际画面 | PASS | rviz-standing.png / rviz-moving.png 已实际截取并查看：Global Status OK，模型/base_link坐标轴位置和前向随运动改变。固定frame=odom；完整模型FK/footprint验证留W07 |
| 发布QoS/消费者/频率 | PASS | consumer-qos.json：实际参数 t1_sim=false、RSP/RViz/probe=true；晚加入订阅仍收到两个static TF及robot_description；joint divisor=2时joints/RSP关节TF实测25 Hz、odom/clock50 Hz；默认均50 Hz |
| 构建/单元测试 | PASS | 系统colcon三包构建exit0；61 tests、0 errors/failures/skipped；c2-tests.json含被测源码哈希 |
| Git/环境/进程 | PASS | source-audit.json / process-audit.json；用户start_commands.md diff SHA256未变；上游固定SHA且clean；自己创建的launch/sim/RSP/RViz/server均wait退出，无剩余PID |

odom是**仿真真值理想里程计**，地面z=0，名义协方差1e-6不代表估计误差。
base_link=trunk XBODY/free joint原点（非质心），base_footprint是竖直投影而非固定杆臂。
IMU首版未发布（可选，无必需消费者）；LiDAR/SLAM/AMCL/Nav2未实现，未声称工作。

## Topic 实测（默认 motion run）

| Topic | Publisher / 类型 | 频率（sim） | stamp范围（s） / QoS |
| --- | --- | --- | --- |
| /clock | t1_sim / rosgraph_msgs/Clock | 50 Hz | 0–23.44；R,V,KL(1) |
| /odom | t1_sim / nav_msgs/Odometry | 50 Hz | 0.02–23.44；R,V,KL(10) |
| /joint_states | t1_sim / sensor_msgs/JointState | 50 Hz（可整数分频） | 0.02–23.44；R,V,KL(10)；23关节 |
| /t1/eval/ground_truth | t1_sim / nav_msgs/Odometry | 50 Hz | 0.02–23.44；R,V,KL(10)；仅测试记录消费者 |
| /tf：base_footprint/base_link | t1_sim / tf2_msgs/TFMessage | 50 Hz | 0.02–23.44；tf2默认R,V |
| /tf：23条关节 | RSP / tf2_msgs/TFMessage | 50 Hz（跟随joints） | 0.02–23.44；tf2默认R,V |
| /tf_static | RSP / tf2_msgs/TFMessage | 启动/持久缓存 | 延迟订阅实测两个stamp=0；R,TL；trunk和head_realsense_rgb_link |
| /robot_description | RSP / std_msgs/String | 启动/持久缓存 | 无header/stamp；R,TL；RViz实际订阅 |
| /imu/data | 无 | 未发布 | 无测量加速度不伪造字段 |

端点类型、Reliable/TransientLocal、实际订阅者与GID见motion-integration.json的graph。
本机RMW端点查询的depth显示0，历史深度以上述源码QoS构造为准，实际连接及晚加入已验证。
Jazzy的订阅MessageInfo无publisher GID；child所有权检查采用实际完整TF batch、URDF唯一child树、端点发布者集合的交叉核对，未把该推断冒充逐消息GID证据。

## 复现

使用未占用的专用TCP发现端口；以下脚本自行起停自己的server，不要在该端口另起server。
所有运行使用scripts/env.sh，系统colcon构建，仿真wrapper显式exec配置venv解释器。

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42116 ROS_DOMAIN_ID=76
export ROS_LOG_DIR=$PWD/run_data/w06_ros_logs
source scripts/env.sh
/usr/bin/colcon build --symlink-install --packages-select t1_mujoco t1_bringup t1_description
source scripts/env.sh
/usr/bin/colcon test --packages-select t1_mujoco t1_bringup
/usr/bin/colcon test-result --all --verbose
python docs/evidence/W06/probe_velocity.py
python docs/evidence/W06/check_model_export.py
python docs/evidence/W06/verify_integration.py --suite controls
python docs/evidence/W06/verify_integration.py --suite motion --rviz
python docs/evidence/W06/verify_consumers.py
```

需本机ROS socket与X11权限。GUI验收对应真实RViz，不只启动进程。
主要run：controls `20261014T082041Z_dev_w06_controls_s0_4c4309`；restart `20261014T082100Z_dev_w06_restart_s0_418413`；motion `20261014T082218Z_dev_w06_motion_s0_89fdf5`；consumers `20261014T082626Z_dev_w06_consumers_s0_be534d`。
原始消息、日志、manifest与CSV保存在忽略的raw/和run_data/，轻量汇总与截图入Git。

## 失败尝试与限制

failed-attempts.json保留失败：沙箱socket权限、测试脚本MessageInfo API假设、整数stamp汇总错误。修正后controls/motion/consumer均PASS；未改依赖以掩盖错误。
单测首次venv无pytest，改用既有系统colcon；首次一项消息array/list断言错误已修正，最终61/61通过。
本步只证明当前场景的状态接口/时间与基本显示；完整URDF↔MJCF FK、场景、LiDAR、定位和导航仍由各后续步骤负责。
暂停/原地reset的轨迹不连续；消费者以episode通知处理，不跨reset积分/插值。其他模式需重启整套进程。

# W10 软件仿真安全链

适用平地、冻结策略/身体几何、W08补偿scan和理想odom。不构成硬实时或人员安全认证。

## 命令与恢复

`/t1/cmd_teleop` 或 `/t1/cmd_navigation`（启动时明确二选一，默认teleop）→ t1_command_gate → `/cmd_vel_nav` → Nav2 velocity_smoother → `/cmd_vel_smoothed` → Nav2 collision_monitor → `/cmd_vel` → t1_sim只减不增保护 → W05映射。全部Twist，enable_stamped_cmd_vel=false。

唯一正常最终发布者为collision_monitor；gate检查三个输出各自只有指定发布者、所选原始源只有一个发布者。无源切换服务，切换须重启。mapping仅允许teleop。W11尚未实现，navigation原始入口用于取消接口试验，不宣称存在导航action。

启动DISARMED。先持续发送所选源零命令，等scan/odom/两个动态TF新鲜、sim READY、smoother和CM active、最终零已持续0.5墙钟秒，再调用 `/t1/safety/arm` Trigger。随后必须发送新命令。`/t1/safety/stop` Trigger用于显式急停/未来W11取消；STOPPED状态锁存，必须满足上述条件后显式arm，不会复活旧命令。NaN/Inf在进入Nav2前锁存；Nav2自身会丢弃非法命令，不能用其替代本门。普通新零命令直接撤销permit，仍经正常话题发零，后续新命令可继续。

permit是仅能否决的Bool，50Hz单调墙钟更新。桥接每20ms策略边界检查permit及其仿真/墙钟双龄≤0.15s，失效清空缓存；不发布第二份/cmd_vel。gate故障或退出也停止。source及scan/odom/动态TF超时均0.30s，双时钟（来自旧stamp的重放不能刷新）；bridge cmd双超时0.50s作为独立兜底。暂停期间gate墙钟继续运行、锁存STOPPED；resume后须显式arm和新命令。所有ROS消息仍用sim time。

OPEN_LOOP smoother：50Hz，vx[0,.75]、vy0、wz±.85，accel[.5,0,1.0]、decel[-.75,0,-1.5]，velocity_timeout=.30。没有未经测试的odom闭环；W05步态响应约0.9–1.8s，.5m/s²是缓慢起步的选择。bridge继续将行走wz裁到±.45，禁止倒退/横移，低于W05死区置零。CM停零和permit撤销均在下一个可用策略周期生效，不受smoother减速度斜坡限制。

## 冻结区域与时延预算

W07最大旋转半径R_body=1.3905394637/2=0.695270m，覆盖手臂/脚/全身保守矩形的任意偏航（包括停车偏航回弹26.1°）。W05最高期望vx=.75m/s，最坏零命令后平移停距.353m。整个圆覆盖前进/转向扫掠，无倒退恢复。

保守半径公式：R_stop ≥ R_body + .75·(t_command + t_scan + t_compute) + d_brake + margin。
取t_command=.50s（bridge兜底，包含gate .30s+lease .15s），t_scan=.10s（10Hz），t_compute=.07s（50Hz CM/末端周期各.02s+调度/扫描预算.03s）；因此无额外裕量为1.550770m。冻结R_stop=1.65m，余量.099230m。该预算是配置上限/预算，不冒称所有项已实测，C3记录实际触发延迟与计算分布；超预算需拒绝验收或扩区重测。软件调度没有硬实时保证。

R_slow=1.95m，额外.30m预警。slowdown_ratio=.5；高端.75→.375仍可走，≤.63经减半落入前进死区则站立，绝不提高最终速度以获得“慢行”。min_points=1，避免忽略细柱；可能保守误停。没有用脚底圆替代全身扫掠。

在静态可见障碍、预算成立时，停止区触发后保守footprint净距下界约.099m；scan_min_range不等于身体三维净距。此圆会限制窄通道可通行性，W11必须尊重并验证，不能为通过导航缩小未重验的安全区。

## 感知和故障

scan需360格、≥75%已知（+inf是未命中，NaN未知）、无-inf盲区回波；保持W08内部健康门。scan/odom/odom→base_footprint及base_footprint→laser_stabilized的新鲜度分别监控。CM还以source_timeout=.30及transform_tolerance=.05拒绝失效输入。

sensor健康、通信故障是零目标+持续站立策略；FALLEN为投影重力z>-.5或trunk z<.50m，FAULT为策略异常/非有限状态/动作等，零命令并停止推进、run失败，必须整套重启，无自动起立。测试倾倒使用有限状态注入，只证明判据，绝不作为物理步行。

瞬时9层射线不保证所有高度/盲区障碍可见；truth姿态补偿、无噪声、有限FOV、ideal odom限制保持。没有安全认证、无动态预测。

## 版本依据

本机nav2_collision_monitor/nav2_velocity_smoother 1.3.13（Jazzy），本地头文件、示例参数与实际get_parameters核对。语义参考官方[jazzy CM源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_collision_monitor/src/collision_monitor_node.cpp)与[smoother源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_velocity_smoother/src/velocity_smoother.cpp)，最终以本机运行证据为准。

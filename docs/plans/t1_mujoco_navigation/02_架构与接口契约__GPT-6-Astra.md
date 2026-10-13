# W02 架构与接口契约

难度：L5 高难。估算投入：2–4小时，含调试与验证，不是保证。
依赖：W01。覆盖需求：R1、R2、R3、R4、R5、R6。遵循决策：D2、D3、D4、D5、D6、D7。
初始状态：TODO；实时状态以[执行看板](执行看板.md)和[本步记录](records/W02.md)为准。
工作类型：用户手动分派后执行本步。一次只做本步，不自动开始后续步骤。

## 进入本步

先读[总规划](总规划.md)、[执行规则](执行规则.md)、[环境定位](环境定位.md)、[当前交接](交接记录.md)、[证据与参考](证据与参考.md)和本步记录。
确认依赖步骤DONE及其代码SHA、验收证据有效。先检查Git差异，保留已有工作；把本步状态置IN_PROGRESS并记录时间、分支、起始SHA。
所有$WS/$PLAN等路径按环境定位解析。本文件中的目标路径是要创建或更新的产物，不代表已存在。
## 目标和产物

把后续实现需要的接口、坐标、线程、时间、速度语义和验收协议一次定义清楚。产物：
- $WS/docs/architecture/system.md。
- $WS/docs/architecture/interfaces.md。
- $WS/docs/architecture/acceptance.md。
- 本规划决策记录的必要补充。此步以文档和少量只读接口探针为主。

## C1：审查真实源码

完整读取 deploy入口、MujocoController、BaseController、ControllerCfg、T1任务配置和实际使用的Locomotion策略类。
列出 start/update_state/policy_step/ctrl_step/reset 对历史观测、动作、关节顺序的要求。
核对 policy_dt、physics_dt、decimation的一致性，确定运行器如何选择时步。
检查模型构造后附加场景的办法；优先通过配置覆盖MJCF或组合运行时模型，不复制整个上游控制器。
识别所有依赖 qpos[7:] / qvel[6:] / ctrl全数组长度的代码点。

## C2：写入接口契约

接口表逐项给出：生产者、消费者、ROS类型、frame、时间、频率、QoS、失效行为。
至少覆盖：
- /clock：唯一来源为仿真data.time；其他节点use_sim_time=true。
- /odom：nav_msgs/Odometry；pose位于odom、twist属于child_frame；首版理想里程计。
- /joint_states：sensor_msgs/JointState；按真实名字映射，非数组索引猜测。
- /scan：sensor_msgs/LaserScan，坐标和姿态处理由W08细化。
- /imu/data：若发布必须说明是真传感器值还是合成值，未知协方差不能伪造高精度。
- /cmd_vel_nav、/cmd_vel_smoothed、/cmd_vel：统一选Twist并显式配置Jazzy参数；若本地组件强制Stamped，要在本步统一并记录，不能混发。
- /diagnostics和可选项目状态话题：状态、超时、跌倒、run_id等。
规定一个进程拥有MjData并推进物理；ROS回调只缓存输入，不与传感器线程无锁并发写物理数据。
明确暂停、重置、时间回跳后所有命令、策略历史、传感器缓存和TF怎么清空。

## C3：确定frame和命令语义

TF建议为 map→odom→base_footprint→base_link→传感器/关节树。
odom→base_footprint只含平面投影，base_footprint→base_link承载高度和roll/pitch。base_link必须与选定真实刚体吻合，不能给摇摆机体一条错误静态TF。
map→odom只由当前SLAM或AMCL提供；两者互斥。动力学真值仅用于理想odom与评测。
区分三种量：物理期望速度、策略输入、实际速度。定义映射配置的归属与单位，禁止把策略输入0.5当成已知实速0.5。
预设只用前进/偏航，是否倒退/原地旋转由W05实测。导航轨迹模型与真实动作集一致。
规定碰撞过滤后无抬升最小速度、无缓慢衰减绕过零速度命令；站立命令保持策略稳定控制，不等于直接断所有关节力矩。

## C4：写验收契约与审查

把总体阈值、故障实验、运行ID、CSV字段、场景种子和证据位置固化；数值未测部分标“待W05确定”，不能编成已知事实。
列出W03–W12分别实现哪些接口及谁拥有TF/地图/命令。逐条检查不存在双发布、环路、重复状态真相和隐含依赖。
输出小型数据流图与控制周期顺序，给出至少三项备选方案及未采用原因：直接修改上游、双线程共同写MjData、纯真值地图导航。
验收：所有后续工作可从文档找到输入输出、停止条件、证据定义；没有“后面再随便选”的关键接口。

## 边界与失败处理

不开始实现整个系统；必要探针只验证函数签名、配置和路径。
若接口涉及现实策略无法支持的能力，列成W05门槛，不在文档中许诺。对重大路线变更先记录ADR及影响步骤。

## 记录、版本控制和中断恢复

检查点C1、C2、C3、C4分别完成后立即更新[本步记录](records/W02.md)，保存已完成/正在做/未完成、命令、退出码、实测值、证据和代码SHA。
原始日志与截图放$WS/docs/evidence/W02/，大文件按执行规则忽略；小型结果摘要需纳入代码Git。
建议本步完成提交主题：`docs(W02): define simulation and navigation contracts`。未完成但需要交接时用`wip(W02): describe checkpoint`，并明确未通过项。
先提交代码/证据，再填写代码SHA并提交规划记录。完成后更新看板和交接，不只在聊天中汇报。
中断时至少记录下一条可执行命令、cwd、解释器/环境、脏文件、仍在运行的进程、最近失败原因及不应重复的操作。若仍有安全相关仿真进程，先进入停止状态。
完成状态要求所有本步验收通过；不能以“代码写完”“暂时跑不起来”标DONE。

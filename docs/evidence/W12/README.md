# W12 动态障碍与传感器闭环证据

W12按单步文件的冒烟门槛验收：每类运行三次，每类至少一次完整成功；所有正式运行保留结果。W13可靠率评测未开始。最终计数和逐项审计见 `final-verification.json`；九例详情见 `smoke-suite.json`、每例 JSON/CSV。`smoke-suite.json.verdict`额外要求九例全部成功，因此包含已记录的导航失败时为FAIL。

运行使用Booster T1官方策略与MuJoCo接触动力学。没有机器人基座位姿行走替代，没有人工中途速度救场。目标用 `navigate_to_pose 9 0 0`，沿用显式arm、分段进近、定位静止核验、.30m/20°/2s真值门槛和120s任务上限。Nav2、安全区及命令整形沿用W11/W10；动态场独立AMCL配置仅将sigma_hit从.1m改为.05m。`static_world`场景、地图及W11证据按起始哈希检查保持。

## 实现与场景来源

- `src/t1_mujoco/t1_mujoco/events.py`：同一owner线程每2ms更新mocap位置/方向；暂停冻结，episode复位后恢复初态。物体有真实碰撞和group2射线几何，不能被机器人推开。
- 动态场YAML仅将三个已登记槽的motion_implemented置true，ADR-017静态几何未改。场景冻结提交 `d9e7ed00724dc0164d2b564f5310fc5b88931002`；之后场景字节不再变化。
- 时刻、速度、触发、种子、允许行为/结束条件在独立 `src/t1_bringup/config/w12_events.yaml`；每次manifest记录路径、SHA256和scenario/seed。场景/地图仍按W09字节哈希绑定。
- crossing：.5m/s、x=2.75，从y−3到y3；机器人开始行走且x≥.6触发。
- blocking：.6m/s进入(4.5,0)，至少保持180s；机器人行走且x≥.25触发，要求到达期间实体不移除且真实绕行。
- approach：.6m/s进入(6,0)、保持3s后退到y3；机器人行走且x≥3.2触发。轴向初始间距约2.55m，保留.245m机器人制动、障碍继续推进及传感器/控制延迟余量；接近体必须撤离以释放B的1.5m进近走廊。

`c1-scene.json`核对真实模型nq30/nv29/nu23，mapping nmocap0、事件运行nmocap1，实际mj_ray命中实体，碰撞位打开；六段建图路线净距最小2.386m。`c1-event-lifecycle.json`为真实步行触发后的ROS暂停/复位复验。

## 独立建图与定位

先提交场景/代码，再通过 `/t1/cmd_teleop`、持续零命令、显式 `/t1/safety/arm` 和W10链真实闭环巡检。mapping不构建动态实体，LiDAR无法把它们烧入地图。

合格地图为 `src/t1_navigation/maps/dynamic_crossing_world.{yaml,pgm,meta.yaml}`，由SLAM Toolbox /map及map_saver生成；没有几何栅格化或手改meta哈希。生成代码 `a5321dbe98228bbaa6f77362e4121751f6a2fec3`，manifest dirty仅用户docs/start_commands.md。巡检27.53m，闭环误差.101m；5cm栅格，墙面P95 .049m/max .073m、完整性P95 .027m、114/114空闲采样点通过W09原阈值。

`mapping-v1/`保留首轮失败：无噪声扫描的北墙端点落到Karto栅格边界外，完整性P95 4.350m。已比较原SLAM-grid和PGM，保存器没有丢格；本机Karto.h的ComputeDimensions与WorldToGrid均Round，height221且north索引221越界。第二轮仅W12 mapping通过独立 `w12_mapping_runtime.yaml`显式启用W08已有1cm射线噪声、seed0，重新实际SLAM建图。噪声写入manifest和meta，导航仍用默认无噪声LiDAR。

`localization-v1/`保留首轮2/3通过、一次扫描对图P95 .168m失败。独立 `w12_localization.yaml`收窄测量似然，其余AMCL参数不变；`localization*.json`三次fresh launch全通过：收敛2.40/2.60/2.68s，位置.012/.017/.043m，偏航.079/.073/.158°，扫描对图P95 .064/.032/.078m。所有W09阈值、初始化偏差、一次initialpose与TF归属要求保持。

## 闭环证据与已记录失败

dynamic-runs.png上排展示全部九条正式轨迹与各自结果，下排速度示例明确标为crossing1/blocking3/approach1。每例JSON包含参数、图/TF归属、真实接触计数、终点核验、扫描命中端点及代价地图采样。`*-trajectory.csv`为10Hz物理轨迹/速度/策略输入；`*-layers.csv`为Nav2→adapter→gate→smoother→Collision Monitor；`*-obstacle.csv`为实体轨迹，`*-costmap.csv`为实体/旧位置占据与命令样本。完整topic数据在ignored `raw/*-topic-data.json`。规划器仅接收scan/costmap，`/t1/eval/obstacle`只被评测器订阅，无未来轨迹输入。

crossing和approach移开后，旧中心占据清为0，机器人无需重arm或人工速度干预即可继续。approach必须在保护区内以W05 .5s位置/偏航窗确认实际站稳≥.5s，再在距离>1.95m时观察恢复行走。blocking必须产生>2.25m实际横偏及重规划，实体保持在(4.5,0)，成功才计入到达。

正式blocking1为ABORTED208：goal段过冲到约(9.50,.35)，进入规划2.05m膨胀区，无法从该起点重规划。blocking2为120.02s到期取消，末端进展失败。两例无接触/跌倒，均保留FAIL并未计成功；blocking3完整真实绕行到达。该类的1/3冒烟结果说明终点进近仍有可靠性问题，不能宣称达到W13的9/10目标。

开发证据也保留：旧harness给整个launch组SIGINT后launch又转发，造成双中断；`event-lifecycle-v1/`及dev日志有退出异常。开发approach虽导航/感知成功，但末尾contact_counts未保存，旧评测把缺失当0，因此证据不足，不计正式完整通过。正式版本改为仅先向launch父进程SIGINT，并要求完整三类接触计数和sim exit0。

动态运行物理代码与评测核心函数冻结于 `5149679a5ae1209991e0c4f62031dc352f59281f`；运行中检查点提交只增加文档/已生成证据，每例manifest保存实际HEAD，审计核对运行源码一致。随后只修正静态回归注册文件路径常量，评测AST核对其余代码未变；静态启动前的路径失败保留。默认S04静态回归的120s超时保留于w12_S04.*。另以static_regression.py验证既有--approach-pass-radius=0选项，等待进近点也超时，独立w12_static_wait_stage.*保留，未采用该参数为新配置。额外运行原W11登记S03绕岛路线作为静态回归，S04两次失败均保留，不重计动态九例。最终代码/证据提交及完成记录SHA见records/W12.md和git log。

## 复现

脚本串行持有DDS端口42122，每次run为独立launch与/clock；不要并行启动。原W11工具和证据未改，复制版本使用W12目录。TF authority可执行文件沿用W09 README编译方法。

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42122 ROS_DOMAIN_ID=32
export ROS_LOG_DIR=$PWD/docs/evidence/W12/raw/ros-log
source scripts/env.sh
/usr/bin/colcon build --symlink-install --base-paths src
source scripts/env.sh
/usr/bin/colcon test --base-paths src
/usr/bin/colcon test-result --all --verbose
"$T1_VENV/bin/python" docs/evidence/W12/check_event_lifecycle.py
"$T1_VENV/bin/python" docs/evidence/W12/dynamic_suite.py --repeats 3 --tag reproduce --rviz
W12_SCENE=static_world "$T1_VENV/bin/python" docs/evidence/W12/nav_run.py case S03
MPLCONFIGDIR=/tmp/t1-w12-mpl /usr/bin/python3 docs/evidence/W12/plot_runs.py
/usr/bin/python3 docs/evidence/W12/final_checks.py
```

上述审计/图默认读取首次正式smoke结果；reproduce套件保存在独立文件名，不覆盖smoke。static回归只写W12证据。重建地图会覆盖dynamic地图，须先保存现有地图/证据并审查Git差异；scene必须已经提交、源码clean，不能改static地图。顺序为remap.py→analyze_map.py→check_localization.py全部通过后才允许动态实验。

交互启动动态案例（需同端口DDS服务已运行）：

```bash
ros2 launch t1_bringup navigation.launch.py scene:=dynamic_crossing_world \
  map:=$T1_WS/src/t1_navigation/maps/dynamic_crossing_world.yaml \
  localization_params_file:=$T1_WS/src/t1_navigation/config/w12_localization.yaml \
  events_file:=$T1_WS/src/t1_bringup/config/w12_events.yaml scenario:=blocking \
  seed:=1202 initial_pose:="0 0 0" rviz:=true
ros2 run t1_bringup navigate_to_pose 9 0 0
```

限制：理想truth odometry及真值姿态补偿，有限垂直射线；反应式避障，无目标追踪预测。mocap障碍不可被推开，停止区不能阻止外界障碍继续进入；报告实测净距和接触，不作硬件安全保证。事件轨迹确定，但MPPI采样/ROS时序允许导航轨迹变化，种子不保证Nav2逐位重放。三次每类仅是W12冒烟规模。

# W11R2 verification log

W11R2 IN_PROGRESS（2026-10-15T09:59:24.829508+00:00）。起始 c37756d680e5d14654f64954f2bdcf3d79d51d7c，初始clean。REPAIR_SHA 166b4a15835a68eb6fd7d1e685f0cc68421f7d77 已冻结；仅5个starter源码/测试文件。受影响t1_bringup构建PASS；完整colcon test 164/164（原151无删除+新增13），0错误/失败/跳过；真实子进程在Client.destroy同步点单次SIGINT退出0，注入RuntimeError保留traceback退出1；真实context先关闭再重复cleanup通过。尚未运行三次完整冷启动关闭；当前无自持ROS/DDS进程，42124/42126空闲。cwd /home/haoyang/humanoid_nav_ws，分支feat/t1-mujoco-navigation；dirty仅本任务W11R2证据与W11记录/看板/交接。下一命令：source scripts/env.sh 后显式venv运行docs/evidence/W11R2/startup_once.py，独立run ID/output，只冷启动关闭，无导航任务。W11R1V失败保持，W13 FAILED，W14/W15 TODO。

补充全仓测试入口检查发现scripts/evaluation/tests有15项独立于colcon的既有测试。首次误用/usr/bin/python3：14通过、1因ModuleNotFoundError: mujoco失败（该依赖只在冻结venv）；完整失败记录保留evaluator-tests-wrong-interpreter.*。不改源码/测试/依赖，下一命令用显式T1_VENV/bin/python并仅在测试进程追加已安装系统pytest路径，按既有W13测试方式运行。未启动正式评测或额外物理运行。


更新时间：2026-10-15T10:09:47.929189+00:00。状态：**CANDIDATE_READY_FOR_INDEPENDENT_VERIFICATION**（仅候选，不是最终导航PASS）。W11R1V INDEPENDENT_VERIFICATION_FAIL原记录保持；W13 FAILED；W14/W15 TODO。当前模型串行，无委派。
起始SHA c37756d680e5d14654f64954f2bdcf3d79d51d7c，初始clean；REPAIR_SHA 166b4a15835a68eb6fd7d1e685f0cc68421f7d77；HANDOFF_SHA为本交接提交，最终回复给出。cwd /home/haoyang/humanoid_nav_ws，分支feat/t1-mujoco-navigation。

精确根因：原startup成功JSON后正常SIGINT打断node.destroy_node→destroy_client→Client.destroy_when_not_in_use；KeyboardInterrupt越过启动try/except和finally，exit−2，尚未rclpy.shutdown。修复仅starter私有context/executor、协作SIGINT与有序幂等cleanup；readiness可选spin回调；异常分类严格，真实RuntimeError继续exit1并保留traceback。新增13测试（含实际starter子进程在client销毁同步点SIGINT），原151项全部保留。
受影响包t1_bringup build PASS；colcon全部4包测试164/164，0错误/失败/跳过；额外既有评测器15/15，总179项不同测试PASS。额外评测器首次误用系统Python缺mujoco导致1FAIL/14PASS，已保留原记录；改用既定venv后15PASS，未改源码/依赖，无物理重试。

连续三次同一REPAIR源码冷启动：
- 20261015T095929Z_w11r2-shutdown-v1-startup_01_s0，14.6731wall/10.00sim秒ready，PASS。
- 20261015T100053Z_w11r2-shutdown-v1-startup_02_s0，14.3486wall/11.68sim秒ready，PASS。
- 20261015T100150Z_w11r2-shutdown-v1-startup_03_s0，14.3202wall/11.58sim秒ready，PASS。
每次全新ROS/DDS/launch，RViz启用；真实8节点active、FollowPath ACK/预期103及2costmap响应；不加sleep，原startup_probe ready后立即关闭。每例starter只收一次SIGINT、exit0；18/18子进程正常退出，无traceback、SIGTERM/SIGKILL升级，wrapper/launch/DDS exit0。环境/障碍/非足触地0、跌倒0；未arm/发导航运动目标。三次共60已观察owned PID全部消失，无遗留ROS/MuJoCo/RViz/DDS/评测进程，42124/42126空闲。未停止无关进程。

源码/脚本指纹4ffaee2ab9b6e31fdf2eb5920ce7ab27c5e78a8b4cbc616949f6ce616d71c28c；REPAIR后源码无变化。W05/W10/导航参数/动态事件/地图/案例/正式评测入口未改；W11R1V 64原始文件哈希和formal-v1 1371哈希全同，上游指定SHA且clean。用户docs/start_commands.md未改/未暂存/未提交；当前提交前dirty仅W11R2新证据和本W11记录/看板/交接，均本任务；提交后要求git status clean。
交接文件docs/evidence/W11R2/REPAIR_HANDOFF.md；证据README、summary、逐run manifest/assessment/退出行号/信号syscall/进程审计和raw哈希齐全。采集脚本原执行路径docs/evidence/W11R2/*.py已移到ignored run_data/w11r2-shutdown-v1/tools，逐字调用文本保存在W11R2/*-invocation.txt，保留内容不改验收。

下一步等待独立验证模型，**必须从头重新执行阶段A到G的完整矩阵，不能续旧STARTUP-02，也不能复用本轮3次样本**。第一条只读命令：`cd /home/haoyang/humanoid_nav_ws && git status --short --branch && git log -8 --oneline --decorate && git rev-parse HEAD`。
未重跑C01/C09/B04/B06或完整物理矩阵；未运行W13 formal-v2、正式RTF验收；未启动W14/W15。当前运行PIDs无。

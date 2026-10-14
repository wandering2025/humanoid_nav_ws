# W13 批量评测证据

最终状态：**FAILED**。完整40例36到达：static10/10、crossing8/10、blocking8/10、sudden10/10；零环境/障碍/非足地面接触、零跌倒。两动态类未达≥9/10；headless严格RTF≥1目标亦未达到（.9999998608），GUI .9991109757达到≥.7。原门槛保留，W14未启动。

综合报告：[evaluation.md](../../results/evaluation.md)、[performance.md](../../results/performance.md)；完整性审计：final-verification.json（证据完整性PASS不等于W13验收PASS）。

入口为 `scripts/evaluate.py`，正式套件 `src/t1_bringup/config/test_suite_v1.yaml`：静态S01–S10、横穿C01–C10、持续阻塞B01–B10、突然接近U01–U10。30组障碍时序在独立w13_events_v1.yaml，不修改地图绑定scene。任务判据沿用W11，含完整2s稳定在120s预算内。

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42123 ROS_DOMAIN_ID=33
source scripts/env.sh
/usr/bin/colcon build --symlink-install --base-paths src
source scripts/env.sh
cmake -S docs/evidence/W09 -B docs/evidence/W09/raw/authority-build -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build docs/evidence/W09/raw/authority-build
# 重跑使用新campaign名，正式结果不要覆盖。
"$T1_VENV/bin/python" scripts/evaluate.py run --campaign reproduce-v1 --limit 10
# 同命令续跑：已有成功/失败均跳过；只有中断/缺失结果追加新attempt，不覆盖旧run_id。
"$T1_VENV/bin/python" scripts/evaluate.py run --campaign reproduce-v1 --limit 10
"$T1_VENV/bin/python" scripts/evaluate.py status --campaign reproduce-v1
# 完成后导出；导出使Git dirty，不要在未提交时继续正式run。
"$T1_VENV/bin/python" scripts/evaluate.py export --campaign reproduce-v1
```

每个case是新Python/ROS context、新DDS server、新全套navigation.launch.py、新策略/clock/TF/AMCL/costmap。run_id为UTC+campaign+case+seed+随机后缀。partial `state.json/summary.json`每例前后原子写入ignored raw；文件锁禁止同campaign并发。源码/配置内容指纹、GUI模式或target_rtf变化必须另建campaign；默认不重试已完成失败，不从分母删除未运行/无效。

正式版本要求代码/配置/地图/套件/评测器均已提交。ADR-018用户允许仅docs/start_commands.md文档dirty例外，每run如实保留dirty及字节哈希。其它dirty仍INVALID。不要暂存或覆盖用户文档。

完整manifest保留simulator原记录，顶层evaluation manifest补充实际目标、case、suite和冻结判据；原manifest SHA另存。包含代码/upstream/checkpoint/MJCF/scene YAML/map image/meta/所有参数文件哈希、真实初态、传感器/odom模式、解释器、时钟、sim/wall、退出/接触计数。每run独立参数、图/TF审计；TF按实际RMW GID去重解析，节点发现阶段unknown不会伪造第二个发布者，两个GID即使同名仍失败。

碰撞为500Hz物理子步接触事件，trunk子树对非地面环境/障碍；脚部地面及自接触不算，非足地面接触失败；同geom对中断≥.04s才新事件。三维body净距是退出后独立模型10Hz重放的MuJoCo collision geom距离，排除地面/visual/self；不是连续净距保证。footprint为10Hz/约2cm边界采样的保守多边形距离，动态体与静态环境取最小；scan最小量程单独报告。

停车事件保存CM stop触发时刻、零策略延迟、零策略后最大平移、W05 .5s窗静止保持2s与恢复时刻。任务接受前的启动停车单独留原始events、不混入任务制动统计。`n_recoveries`来自NavigateToPose反馈number_of_recoveries（按goal UUID取最大后求和）；`n_replans`为/plan更新数减首条，包含分段目标换目标，日志失败分类另保留。

性能测量记录每策略步policy/physics总计算ms、扫描due耗时及10s墙钟RTF。导航进程/proc CPU约.5s墙钟窗，单位CPU ms/wall-s，包含线程开销；不是回调延迟。正式可靠率target_rtf=1；独立节拍测量headless使用T1_EVAL_RTF=1.1，GUI使用1.0（不声称极限容量），GUI专用--gui同时打开MuJoCo viewer/RViz并截实际窗口，不混入可靠率分母。没有改dt/decimation、安全区或导航参数。

开发pilot-v1保留布尔CSV解析/旧文档哈希错误的INVALID；pilot-v2保留TF同GID发现阶段误判；pilot-v3为最终评测器预检。它们不计正式40例。最终正式和专用性能结果见各campaign，综合报告 `docs/results/evaluation.md`、`performance.md`、`results.csv`、`failures.csv`、`audit.json`。

```bash
# 新终端配置同上，先保证Git只剩用户文档dirty
T1_EVAL_RTF=1.1 "$T1_VENV/bin/python" scripts/evaluate.py run --campaign perf-new-headless --cases S04
T1_EVAL_RTF=1.0 "$T1_VENV/bin/python" scripts/evaluate.py run --campaign perf-new-gui --cases S04 --gui
# 已安装系统pytest仅在测试进程追加，不安装/升级依赖
"$T1_VENV/bin/python" -c 'import sys; sys.path.append("/usr/lib/python3/dist-packages"); import pytest; raise SystemExit(pytest.main(["-q", "scripts/evaluation/tests"]))'
# 原始首次campaign全部仍在时，重建首次结果报告：
MPLCONFIGDIR=/tmp/t1-w13-mpl /usr/bin/python3 scripts/report_evaluation.py
```

脚本只SIGINT自己启动的launch父进程，让launch转发一次；超时仅终止自持进程组，收集children与authority退出，并核对已观察的PID/starttime无遗留。不要kill所有ROS/Python。所有大型日志、geometry和原trace均在ignored raw/run_data，本目录只追踪轻量证据。

正式40例完成后才修复启动失败分类和导出格式：原始worker JSON/CSV不改，最终结果表由report_evaluation.py统一离线分类，C01 INVALID→FAIL_STARTUP，成功数不变。附加CM stop/slowdown相关零策略位移来自同批原始trace，不能解释为受控制动实验。RTF聚合使用完整10s墙钟窗；逐run旧worker统计排除前10s仿真，原值仍保留。
批量evaluation由scripts/evaluate.py编排完整navigation.launch.py，底层真实mode=navigation；不使用尚未接通的mode:=evaluation启动别名。

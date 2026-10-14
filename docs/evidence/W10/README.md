# W10 速度安全链与故障停车证据

本步仅验证命令安全链，不实现到点导航。真实运动由冻结官方策略/关节力矩/MuJoCo接触动力学产生；有限状态注入仅用于F12/F13判据，不当作行走。

## 复现

cwd `/home/haoyang/humanoid_nav_ws`。使用系统colcon与显式venv；不修改上游或依赖。

```bash
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42120 ROS_DOMAIN_ID=30
export ROS_LOG_DIR=$PWD/docs/evidence/W10/raw/ros-log
source scripts/env.sh
/usr/bin/colcon build --symlink-install --base-paths src
source scripts/env.sh
/usr/bin/colcon test --base-paths src
/usr/bin/colcon test-result --all --verbose
# 脚本自己启动/退出专用DDS server，不要另起同端口server。
$T1_VENV/bin/python docs/evidence/W10/run_checks.py normal source_timeout nan scan odom tf pause cancel monitor_exit gate_exit limits wall deadband_wall
$T1_VENV/bin/python docs/evidence/W10/runner_faults.py
# 复用W09只读RMW TF authority探针；先构建（输出ignored）。
cmake -S docs/evidence/W09 -B docs/evidence/W09/raw/authority-build
cmake --build docs/evidence/W09/raw/authority-build
$T1_VENV/bin/python docs/evidence/W10/mode_checks.py
$T1_VENV/bin/python docs/evidence/W10/audit.py
$T1_VENV/bin/python docs/evidence/W10/final_checks.py
```

运行期需要本机DDS TCP socket权限；由启动脚本保存每个进程PID，结束只清理本次启动的进程。原始50Hz trace/manifest在ignored `run_data/w10_*`，launch日志在ignored `raw/`。停止probe时先发送源零/stop服务，等待站稳，然后SIGINT该launch。独立运行器异常试验自动终止。

## 验收对象与文件

- `start-state.json`：起始分支/完整HEAD/用户脏文件diff哈希。
- `safety.yaml` 在 `src/t1_navigation/config/`；推导/状态机/局限见 `docs/architecture/safety.md`。
- `w10_<case>_<id>.json`：独立launch命令、被测SHA、图/实际参数、注入仿真及墙钟、三层命令+实速/状态时间线、停距、逐项检查、退出码与PIDs。
- `fault_tests.csv`：矩阵索引；`physical-audit.json`：从真实50Hz trace重新计算的2s站稳、停距、实际延迟、计算耗时、命令不放大检查。
- `runner-faults.json`：高度/倾角、策略异常、非有限动作/状态，验证FALLEN/FAULT、u=0、物理停止和不可自动恢复。
- `mode-regression.json`：带安全链的mapping/localization真实图与RMW发送者身份；确认SLAM/AMCL仍独占map→odom、truth隔离、debug关闭。
- `final-verification.json`：最终版本/构建与测试/上游和用户diff/进程清理审计。

## 故障方法

| 规范ID | 实验case | 注入与预期 |
| --- | --- | --- |
| F01 | source_timeout | 停止所选源发送；双龄超时，撤销permit，零目标站立；锁存 |
| F02 | nan | 所选源发送NaN；进入Nav2前立即锁存，持续后续非零不能自行恢复 |
| F03 | limits | 请求20m/s、20rad/s，先上游裁剪再末端W05组合限幅；无倒退/横移/速度抬升 |
| F04 | pause | 有运动时pause，墙钟仍检测，恢复不复活旧源命令，显式arm才可再动 |
| F05 | 每例清理 | SIGINT自己launch，sim退出0、子进程不遗留 |
| F06 | scan | relay停止转发真实scan；内部仍生成scan，证明外部健康门/CM超时真正起效 |
| F07 | odom / tf | 两例分别停止转发真实odom和TF，不把一个故障冒称两个独立试验 |
| F08 | monitor_exit | 给唯一CM子进程SIGINT，bridge超时/permit门归零 |
| F09 | cancel | 选navigation原始源，在其仍持续非零时调用stop取消；旧源不能复活。尚无W11 NavigateToPose action，真实action取消需W11集成验收 |
| F10 | wall | 实际走向静态墙，先slow再stop；记录CM触发和零后实际停距、保守身体净距界 |
| F11 | deadband_wall | .5请求经CM减半到.25，末端u=0，不抬升至起步速度 |
| F12 | height / tilt | 独立runner有限高度/70°倾角注入，FALLEN失败终止；不是自动起立试验 |
| F13 | policy_exception / nonfinite_action / nonfinite_state | 独立测试钩子，无生产故障注入接口；FAULT且物理不推进 |
| 额外 | gate_exit | watchdog进程退出，独立permit双龄0.15s过期，不能因watchdog故障继续走 |
| 正常 | normal | 完整生产launch，前进、零停、正负原地转向；保护不是永久锁死 |

`fault.launch.py`仅用于外部传输故障试验，通过launch scoped remap+只读relay中断scan/odom/TF，物理与传感器真实生成不变。wall/normal用正式production launch，无relay。所有case预先固定，失败尝试保留且不会计PASS。

## 保留的开发失败

首次command_gate使用rclpy Node只读属性名clients，启动失败且无permit，机器人不动；改名lifecycle_clients。另一次连续重启中GetState请求无响应导致DISARMED，增加0.3s墙钟查询超时/重试，没有绕开active门。对应`*-error.json`与raw日志。首次shell重定向因raw目录缺失未启动任何试验。首次最终mode回归中，证据脚本调用initial_pose缺x y yaw位置参数，argparse退出，AMCL未初始化导致localization unique_map_tf=False（`mode-regression-initialpose-error.json`）；脚本改传W09第1轮近似位姿0.2 -0.15 0.15并检查其退出码后重跑PASS，production代码未改。用户在本步执行中继续编辑docs/start_commands.md，`final_checks.py`改为核验该文件未被任何W10提交或暂存，并记录起止diff哈希。

## 适用限制

软件级仿真保护；不是硬实时/真机/人员安全认证。ideal truth odom、truth姿态补偿、无噪声瞬时scan与有限垂直束仍为前提，不能保证所有高度障碍可见。1.65/1.95m圆比较保守，W11须验证可达性。W10导航取消为命令源接口验收，不能表述为已有自主导航。旧W09直发/cmd_vel的开发脚本不再适用于安全launch，W09地图与定位证据本身不被改写。

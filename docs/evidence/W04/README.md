# W04 验收证据

W04 C1–C4 PASS；未开始 W05。固定被测实现：`4c0d674f2662de346b81a9bd9f4d62c5981093ac`。最终证据提交不修改该版本的 src/scripts；`final-verification.json` 保存所有实现文件 SHA256、实际安装模块与源码字节一致断言、每条命令/UTC/墙钟/退出码。用户原有 docs/start_commands.md 保留，因此 manifest 如实 dirty=true；这是开发验收，不是 W13 正式导航评测。

## 结果

| 验收项 | 结果与证据 |
| --- | --- |
| 原行为与索引/PD | `c1-baseline.json` 保存23关节地址/执行器、默认姿态、增益/限幅及21策略关节顺序；`adapter.md` 说明复用点 |
| 原始场景运动对照 | 5 s站立+10 s策略命令0.5；官方前进3.73705 m；适配器全部qpos/qvel轨迹最大误差 **0**，`check-runner.json` |
| 无界面稳定性 | 独立进程3×60 s仿真，各3000策略步，无NaN/跌倒，末姿态完全一致；最低高度0.61956 m，最大投影重力z=-0.99858；`standing-results.json` |
| 基准 | 无界面控制循环RTF中位数 **18.51**（不含进程/模型加载），三次约18.18/18.51/18.94；总墙钟也分别保存在manifest。此为原场景开发基准，未测W13完整导航负载 |
| 输入与暂停 | 连续策略输入、双超时、非法值及恢复、裁剪、未标定/cmd_vel拒绝、暂停clock不发、暂停期间消息丢弃、恢复旧命令不复活；`controls-results.json`及4个纯逻辑单元测试 |
| 复位与时间 | reset清历史/last_action/filter/计数/命令，episode+1，clock无回跳；13项物理/故障检查及ROS reset实测，`check-runner.json`、`controls-results.json` |
| 故障 | NaN/Inf动作、非有限状态、策略异常、跌倒、MuJoCo隐式复位检测停止物理；不兼容DOF模型、错误模式/use_sim_time/超时参数启动拒绝exit2；`startup-failures.json` |
| 单一物理写入者 | 主线程持有MjData；回调仅缓存/排队；越线程调用实测拒绝；viewer使用独立model/data快照 |
| 退出与stdin | 无stdin运行；headless SIGINT/SIGTERM exit0；真实GUI SIGTERM与窗口WM_DELETE_WINDOW均exit0，GUI控制RTF约1.005；`gui-sigterm.json`、`gui-window.json`、`gui-viewer.png` |
| 包与真实入口 | 系统colcon四包构建、20 tests无失败；simulation.launch及bringup enable_sim真实入口exit0；`final-*.txt` |
| 上游与清理 | 两上游SHA/clean不变，本次仿真、viewer、callback线程、发现服务器均退出；`acceptance-summary.json` |

所有运行的仿真时间和墙钟同时记录。原始CSV/events/stdout保存在被忽略的run_data及raw，轻量JSON/摘要/实际截图受Git管理。无LiDAR、odom/TF、SLAM或导航实现声明。

## 复现

```bash
cd /home/haoyang/humanoid_nav_ws
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1
export ROS_DOMAIN_ID=74
export ROS_LOG_DIR="$PWD/run_data/w04_ros_logs"
source scripts/env.sh
colcon build --base-paths src
source scripts/env.sh
colcon test --base-paths src
colcon test-result --verbose
python docs/evidence/W04/baseline.py < /dev/null
python docs/evidence/W04/verify_all.py
```

verify_all自行启动/清理专用loopback TCP discovery server（默认42114），不要预先在同端口启动另一台server。每条子命令最长60墙钟秒。需要本地DDS sockets和GUI显示权限；窗口证据使用本机已安装xwininfo/ImageMagick/libX11，不安装新依赖。它不会停止别人的ROS/Python进程。

人工交互用法见根README：在一个专用终端运行`scripts/start_local_dds.sh`；每个ROS终端先设置`T1_LOCAL_DDS=1`再source scripts/env.sh。

## 已修复故障与限制

- 初次诊断发布将octet字段当整数导致SIGABRT；按本机生成消息定义改为DiagnosticStatus常量，20单测及真实发布复验通过。
- 初次GUI退出SIGSEGV(-11)单独保存在`gui-failed-segfault.json`。读取安装版viewer.py后，关闭时join本次renderer线程，SIGTERM和窗口关闭复验均0；未用manifest中的0掩盖实际-11。
- 默认本机UDP发现路径仍不可用，保留`controls-failed-*.json`及官方C++对照；已验证显式TCP发现路径，见[调查](dds-investigation.md)和ADR-009。未修改系统网络、.bashrc、依赖或冻结上游。默认环境未被静默覆盖。
- warmup=2 s、双超时=0.5 s为开发初值；物理速度映射、死区、转向/停距/停车包络由W05/W10验证。当前非零/cmd_vel被拒绝，只能显式debug策略命令运动。

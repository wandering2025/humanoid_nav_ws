```bash
./scripts/run_demo.sh
```

**W14_DEMO_READY_WITH_DDS_TEARDOWN_WARNING**。这是唯一推荐的完整现场入口，默认 GUI 已解除本次 DDS-only 封锁。
脚本自动定位仓库、加载 Jazzy/当前 overlay/既有 venv，检查资源/端口/遗留进程，启动 DDS、MuJoCo/T1、
真实 LiDAR、AMCL、Nav2、安全链、动态障碍和 NVIDIA RViz；真实就绪后自动发送 A→B，无需多终端或手动 source。
在其他 cwd 可直接执行 `/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh`。

到达后安全停车并保留双窗口供讲解；按 Ctrl+C 正常结束，等待独立摘要和端口冷却完成再关闭终端。
日志/原始 result/独立 final-result 保存在 `run_data/demo/<run_id>/`；物理记录在 `run_data/<run_id>/`，均 ignored。

**DDS_TEARDOWN_EXCEPTION** 只接受完成后的关闭警告，全部条件见[独立发布决策](dds-teardown-exception.md)：

- 必须已导航 SUCCESS、动态判据 PASS、零碰撞/跌倒，安全停车 ACK 与物理静止均完成。
- 目标工具、launch、MuJoCo、RViz、navigation_starter 与所有机器人组件均退出0；DDS运行期未死亡、ROS通信未中断，且只在预期SIGINT之后退出。
- DDS日志、哈希及真实退出码均保存；不需SIGTERM/SIGKILL；DDS PID消失、端口释放、无项目残留、无宿主图形故障或黑屏。
- 此时终端醒目显示 `DDS_TEARDOWN_WARNING dds_returncode=-11`（示例），整体返回成功；JSON仍保留真实非零退出码。

任何缺项、运行期DDS死亡/通信丢失、导航或停车失败、其他组件异常、强杀、残留、宿主故障仍为 FAILED，不自动重试。
第二轮原始 FAILED/−11 永久保留；新汇总为 NAVIGATION_SUCCESS / SAFETY_PASS / DDS_TEARDOWN_WARNING，详见[清单](release-manifest.md)。

本机依赖见[环境定位](../plans/t1_mujoco_navigation/环境定位.md)。不自动安装/升级，不改变驱动或系统设置。
MuJoCo和RViz明确选择NVIDIA RTX4050，WSLg实际使用Mesa/D3D12 NVIDIA；策略保持CPU。
RViz使用RGB点格和Qt线程正常析构薄宿主，禁用原MapDisplay shader。监控不是宿主故障隔离保证，原黑屏事故仍披露。

辅助接口：`--help`、`--preflight`（CPU资源检查，不启动ROS/DDS/图形）、`--headless`、`--scenario U01`、`--no-rviz`。
固定formal-v1 U01/seed4000，A(0,0,0)→B(9,0,0)，是成功固定演示，不是40例评测或随机可靠率统计。
已有两轮约30–42秒至ready，任务46.66–47.30秒仿真时间；保留最终画面后Ctrl+C，TCP冷却可另需约60秒（最多65秒）。
首次W13仍FAILED：36/40、crossing/blocking各8/10、严格headless RTF略低于1。首次黑屏、CPU失败、RViz−6、DDS−11证据均保留。
录像 **FINAL_VIDEO_PENDING_W15**，尚无已核验可播放备用片；本次不启动W15或新增GUI排练。
[5–8分钟流程](live-demo-runbook.md) · [排障](troubleshooting.md) · [架构限制](architecture-and-limitations.md)。

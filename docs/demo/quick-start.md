```bash
./scripts/run_demo.sh
```

**DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**。在 Ubuntu-24.04 的 `/home/haoyang/humanoid_nav_ws` 中执行这一条命令。无需手动启动DDS、source多个终端、点击目标或驱动机器人。

默认固定U01/seed4000，A(0,0,0)→B(9,0,0)。脚本加载既有ROS Jazzy/overlay/venv，先启动MuJoCo与安全/导航，再启动RViz；就绪后自动导航。橙色障碍接近路线，机器人安全等待，障碍退开后继续到绿色B点。

- MuJoCo：青色A START、绿色B GOAL；真实T1步行，跟随视角。
- RViz：SLAM生成的地图、红色LiDAR点、蓝色规划路径、局部障碍代价、安全边界和A/B。窗口重叠时可用Alt+Tab切换，不需要第二条启动命令。
- 正常结束：到达后安全停车，保留画面8秒，再自动关闭登记进程。等待终端显示 `Final SUCCESS` 或符合严格条件的 `SUCCESS_WITH_WARNING`。TCP端口冷却另需至多65秒，不要提前关闭终端。
- 讲解需要更久时可使用 `--hold`，最终画面保留到Ctrl+C。
- 紧急停止：在启动终端按一次 **Ctrl+C**，等待锁存停车、物理静止、进程清理。不要用全局pkill杀ROS/Python，不要直接关终端。
- 新图形故障会锁存并停止；不要删除锁或自动重试。现场改用下面的已验证录像，保存日志。

CPU预检接口为 `./scripts/run_demo.sh --preflight`；它不启动仿真或GPU。入口不安装/升级依赖，不改变驱动或显示设置。适用环境是本机已冻结的Ubuntu-24.04/WSLg/NVIDIA RTX4050配置；并非空白系统安装器。

[最终录像](../evidence/W15/raw/recovery/final-recording-02/demo.mp4)：77.125秒，1024×736、8fps H264，无音频，1896620 bytes。617/617帧软件解码及关键画面检查通过。
SHA256 `9980c060692af05c26f6ed83ac2a3a5b5dd88935a4cc72de8fdd1966470e0df0`。
录像前69.125秒是同一次运行的连续真实窗口抓取、保持墙钟时间；最后8秒明确标注为运行后证据卡。RViz独立截图在[证据清单](release-manifest.md)。

最终录像运行 `20261015T155946Z_demo_U01_79a6bb62-5b16-427f-986d-eb23df9a93c1`：SUCCESS、动态PASS、碰撞0/跌倒0、停车/清理通过，全部组件退出0。日志位于 `run_data/demo/<run_id>/`，物理trace位于 `run_data/<run_id>/`，原始数据均ignored并有Git追踪的哈希索引。

已批准的[DDS关闭例外](dds-teardown-exception.md)保持全部14项条件，真实退出码仍保存，终端必须显示警告。运行期DDS、导航、停车、其他组件或宿主图形故障均不能归入该例外。
W13正式 **FAILED/36/40** 不变；本版本是固定场景演示，不是可靠性认证。

[演示流程](live-demo-runbook.md) · [讲解提纲](presentation-outline.md) · [架构与限制](architecture-and-limitations.md) · [完整版本清单](release-manifest.md)。

```bash
/home/haoyang/humanoid_nav_ws_advanced/scripts/run_advanced_demo.sh
```

在本机已构建的独立 worktree 运行这一条命令。无需遥控、点击目标、手工启动 DDS 或逐个 source 终端。最终验收状态、录像和运行证据见 [release-manifest.md](release-manifest.md)。

机器人从青色 A 出发，自主执行 9 个建图航点，经西区、北侧宽廊、东区后回到 A；保存实际 SLAM 地图，再绕过隔墙前往绿色 B。最终导航时橙色障碍接近、停留 6 秒并撤离；机器人利用 LiDAR/costmap 重新规划并继续。固定路线建图称为 **guided autonomous mapping**，不是 frontier exploration。

- MuJoCo 显示物理步行、空间结构、A/B 和障碍；RViz 显示红色 LiDAR、逐渐增长的 SLAM 地图、蓝色规划路径、局部障碍代价和安全边界。可用 Alt+Tab 切换窗口，不需要操作机器人。
- 完整流程约 7–9 分钟，取决于本机负载；到达后验证停稳并保留画面 8 秒。等待终端显示 `Final SUCCESS`。进程退出后的 TCP TIME_WAIT 冷却另需至多 65 秒，保持终端打开。
- 需要讲解最终画面时加 `--hold`，随后按一次 Ctrl+C 完成清理。任何阶段按一次 **Ctrl+C** 都请求锁存停车；等待物理停稳和登记进程退出。不要全局 pkill ROS/Python，也不要直接关闭终端。
- `--preflight` 只做 CPU 预检；`--headless` 执行相同物理任务但不启动窗口。录像为独立流程，正常演示入口不会自动录制。
- 若出现新的图形错误或持续无响应，监控会停止最近启动的登记图形任务并保留日志。不要删除故障锁反复尝试；使用已验证录像或稳定版。

稳定版兜底仍在原工作区：`/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh`。不要从高级 worktree 调用旧入口；高级版本从未替换原工作区的默认演示。

本机环境为 ROS 2 Jazzy、MuJoCo 3.15.0、冻结的 T1 policy/资产/venv、WSLg 与 NVIDIA RTX 4050。入口不安装依赖、不改驱动或显示配置。它不是全新机器的安装程序。

[讲解提纲](presentation-outline.md) · [架构与限制](architecture.md) · [验证与录像](release-manifest.md)

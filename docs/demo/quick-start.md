```bash
./scripts/run_demo.sh
```

这是唯一推荐的完整入口。当前正在W14正式排练，未宣布DEMO_READY；先看[发布清单](release-manifest.md)的最新状态。
脚本自动定位仓库、加载Jazzy/当前overlay/既有venv、检查冻结模型/权重/地图、端口和遗留进程；
启动本地DDS、MuJoCo/T1、ROS桥、LiDAR、AMCL、Nav2、安全链、U01动态障碍和RViz，就绪后自动发送A→B。
不需要source环境或另开终端。其他cwd可用 `/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh`。

本机Ubuntu24.04/WSL2；ROS Jazzy与冻结依赖须已存在，默认路径见[环境定位](../plans/t1_mujoco_navigation/环境定位.md)。
不自动安装/升级依赖，不修改驱动或系统显示。缺少overlay时按README开发构建说明使用系统colcon构建。
MuJoCo明确选择RTX4050（WSLg OpenGL仍通过Mesa/D3D12 NVIDIA后端）；策略保持原CPU数值路径。
RViz使用Landlock拒绝GPU设备的llvmpipe软件渲染；地图/costmap由原始网格转换RGB点格，仅输出 `/t1/viz/*`。
原MapDisplay索引纹理shader已禁止，不以切回Intel或更新驱动重试。监控不构成宿主故障隔离保证。

固定formal-v1 U01/seed4000，A(0,0,0)→B(9,0,0)，不改参数；不是40例评测或随机可靠率。
约20秒就绪（另有NVIDIA空闲窗口/导入预检耗时），导航约一分钟；实际以run摘要为准。
到达后安全停车并保留双窗口，终端提示按Ctrl+C结束；随后确认最终SUCCESS、退出码和端口释放。
TCP冷却最多65秒，不要因此重复启动。启动就绪/任务均有超时，失败不自动重试或换seed。

接口：`--help`、`--preflight`（CPU资源预检，不触碰GPU/不启动ROS）、`--headless`（CPU辅助，无窗口）、
`--scenario U01`、`--no-rviz`、`--startup-check`（只站立/停车/退出，不导航）。
日志/原始result/独立final-result：`run_data/demo/<run_id>/`；物理trace/manifest：`run_data/<run_id>/`。
详细日志留文件，终端每5秒显示状态。只回收本次PID/starttime，不杀其他ROS/Python。

首次W13仍FAILED（36/40），CPU辅助动态停车0.42s失败与图形事故全部保留。
[现场流程](live-demo-runbook.md) · [排障](troubleshooting.md) · [架构限制](architecture-and-limitations.md)。

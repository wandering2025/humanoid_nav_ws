```bash
./scripts/run_demo.sh
```

这是唯一完整入口，但当前 **W14_BLOCKED**：本轮RViz三次退出0，但第二轮DDS关闭发生SIGSEGV（−11），GUI已封锁；命令返回3且不访问GPU。
不要直接清除halt重跑相同路径。当前不是可发布现场版本，详见[发布清单](release-manifest.md)。

已实现：从脚本自身定位仓库，自动加载Jazzy/当前overlay/既有venv，检查Ubuntu24.04、冻结依赖/模型/权重/地图、
端口/遗留进程，统一启动DDS、MuJoCo/T1、桥接、LiDAR、AMCL、Nav2、安全链、动态障碍和RViz；
真实就绪后自动发A→B。到达后停车、保留画面，Ctrl+C清理本次PID，独立核验端口。没有改导航或安全参数。
恢复审查通过后无需多终端/手动source；其他cwd可用 `/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh`。

本机依赖位置见[环境定位](../plans/t1_mujoco_navigation/环境定位.md)，不自动安装/升级依赖或修改系统。
MuJoCo明确选RTX4050，WSLg仍用Mesa/D3D12 NVIDIA；策略保持CPU。RViz为明确NVIDIA的RGB点格与Qt线程正常析构薄宿主；
原MapDisplay shader禁止。监控不构成宿主故障隔离保证。

接口：`--help`；`--preflight`（CPU资源检查，不启动ROS/DDS/图形）；`--headless`（CPU辅助，无窗口）；
`--scenario U01`；`--no-rviz`；`--startup-check`（无导航站立检查）。当前所有GUI模式均因DDS关闭失败及授权次数耗尽的halt拒绝。
CPU辅助已有一次动态判据FAIL，不循环重跑刷成功，也不替代两次默认排练。
日志/原始result/独立final-result：`run_data/demo/<run_id>/`；物理trace/manifest：`run_data/<run_id>/`，均ignored。

固定formal-v1 U01/seed4000，A(0,0,0)→B(9,0,0)，仅为演示，不是完整40例评测或随机可靠率。
本轮完整GUI约30–42秒至ready、约46.66–47.30s任务sim；退出后的TCP冷却可另需约60秒，最多65秒。
端口冷却时等待，不为TIME_WAIT杀进程。失败不自动换seed或重试，关键组件启动有界超时。
首次W13仍FAILED（36/40）；全部启动失败、CPU失败、图形事故和RViz异常退出均保留。
[现场流程](live-demo-runbook.md) · [排障](troubleshooting.md) · [限制](architecture-and-limitations.md)。

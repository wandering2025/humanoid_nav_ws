# 一行入口（当前封锁）

```bash
./scripts/run_demo.sh
```

**W14_BLOCKED：该命令当前仅报告事故并返回3，不启动任何组件。不要绕过封锁。**
首次默认 GUI 排练出现宿主黑屏并强制重启，见[安全审查](graphics-incident-20261015.md)。
续作后 `--preflight`、`--headless`已通过按进程Landlock设备限制开放：无GPU设备访问、无窗口、无CUDA探测。
`--no-rviz`仍会请求MuJoCo窗口，因此单独使用继续封锁。`--help`可查看接口。
CPU预检命令为 `./scripts/run_demo.sh --preflight`；它不启动ROS/DDS节点。
`./scripts/run_demo.sh --headless`仅为辅助检查，已测一次到达但动态停车持续时间不达冻结判据，整体仍FAIL；不要循环重跑刷成功。
[恢复方案](cpu-recovery-plan.md) · [实测证据](../evidence/W14/cpu-recovery/README.md)。

设计目标：脚本从自身位置找仓库，自动加载 scripts/env.sh 中的 Jazzy、install overlay、
配置 venv；统一启动 DDS、MuJoCo、T1、LiDAR、AMCL、Nav2、安全链、动态体和 RViz。
这是已提交实现的意图，**尚未通过完整启动、到达、画面及安全退出验收**。
其他目录可调用绝对路径 `/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh`。

固定案例 formal-v1 U01/seed4000，A(0,0,0)→B(9,0,0)，不修改原时序、参数或阈值。
该案例历史成功不保证 GUI/ROS 调度下每次成功；不是40例可靠率测试。
日志 `run_data/demo/<run_id>/`；trace/manifest `run_data/<run_id>/`，均 ignored。
本机依赖绑定见[环境定位](../plans/t1_mujoco_navigation/环境定位.md)，版本见[发布清单](release-manifest.md)。
W13 保持 FAILED（36/40）；W15 未启动。解除事故阻塞必须另行审查和授权。

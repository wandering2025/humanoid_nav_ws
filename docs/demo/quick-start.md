```bash
./scripts/run_demo.sh
```

**当前：FINAL_RELEASE_BLOCKED。** 这是唯一完整演示入口，但当前在打开GPU/GUI前返回3。请勿删除`run_data/demo-gpu-halt.json`或改profile绕过封锁。W15最小显示检查发生NVIDIA用户态图形库崩溃，完整最终GUI和成功录像均未通过；不能用于求职终面现场演示。

本机目录：`/home/haoyang/humanoid_nav_ws`，Ubuntu-24.04。入口自动加载`env.sh`中的ROS Jazzy、当前overlay与既有venv，不需多终端或手动source，不安装/升级环境。可用`./scripts/run_demo.sh --help`查看接口；`--preflight`只检查CPU资源与本地端口，不开启仿真或图形。

已实现的操作接口（最终GUI验收未完成）：

- 启动：上述唯一命令。原固定U01/4000，A(0,0,0)→B(9,0,0)，保持真实步行、LiDAR和安全链。
- 正常结束：到达后执行安全停车，画面保留8s后自动清理。`--hold`用于保留最终画面讲解，再按Ctrl+C。
- 紧急停止：在拥有演示的终端按一次Ctrl+C，等待锁存停车、物理静止、登记进程回收和最终结果。不要关闭终端后任其运行，不要pkill所有ROS/Python。
- 日志：`run_data/demo/<run_id>/`；物理trace：`run_data/<run_id>/`。TCP冷却最多65s。`SUCCESS_WITH_WARNING`只能用于满足全部14项条件的[DDS关闭例外](dds-teardown-exception.md)，任何MuJoCo/RViz/宿主图形错误都不能归入此例外。

没有已核验成功原片；两段minimum诊断视频仅用于解释失败，不可作为备用演示。录制工具`/usr/bin/python3 scripts/demo/record_demo.py --output <新的目录>`会执行原命令、实时捕获两个真实窗口、用现有CPU H264编码；当前同样会遇到入口封锁，尚未获得发布录像。

[最终审查](../results/final-review.md) · [完整版本/失败证据清单](release-manifest.md) · [技术解释草案](presentation-outline.md)。W15以BLOCKED结案，无W16、无发布tag、无自动后续任务。

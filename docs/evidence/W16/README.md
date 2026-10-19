# W16 运行与验收

状态以 [W16 执行记录](../../plans/t1_mujoco_navigation/records/W16.md) 为准。

在 Ubuntu Bash 中，从 `/home/haoyang/humanoid_nav_ws_advanced` 执行：

```bash
./scripts/run_exploration_demo.sh
```

Windows PowerShell 中使用：

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws_advanced --exec bash ./scripts/run_exploration_demo.sh
```

默认 headless，持续打印进度，结束写 final-result.json。Ctrl+C 请求锁存停车并清理本次进程，等待脚本返回；DDS 退出后本机 TCP 回收可能额外等待约65秒。

编辑 `src/t1_navigation/config/exploration_task.yaml` 中的 x/y，即 B 在探索开始时 A 身体系中的米制偏移。A 的 +x 向前、+y 向左；输入不是固定 map 绝对坐标，不指定终点 yaw。运行开始会保存任务快照，中途改文件仅影响下次。原 `presentation/goal.yaml` 属于旧 guided 演示，本入口不读取它。

```bash
./scripts/run_exploration_demo.sh --preflight
./scripts/run_exploration_demo.sh --task /absolute/path/task.yaml
./scripts/run_exploration_demo.sh --scene /home/haoyang/humanoid_nav_ws_advanced/src/t1_mujoco/worlds/exploration_south.yaml
./scripts/run_exploration_demo.sh --task src/t1_navigation/config/exploration_unreachable_task.yaml --time-limit 20
```

最后一个是预先登记的失败用例：B 在测试世界之外，20仿真秒任务预算后应失败并安全停车，不能统计为成功到达。场景参数只进模拟器，不传给探索进程。

新环境不在本步范围。已有 overlay 若需重建：设置既有 `T1_VENV`、`T1_UPSTREAM` 后 `source scripts/env.sh`，用系统 `/usr/bin/colcon build --symlink-install --base-paths src`；运行入口明确使用配置 venv Python，不重装依赖。

证据：cpu-checks.json、source-audit.json、attempt-*.json。原始 run_data/exploration/每个ID包含决策日志、在线地图快照、ROS日志、manifest及最终结果；物理trace在 run_data/同ID。失败保留，运行通过和图形验证分开；W16未运行GUI/录像。

详见 [探索架构](../../architecture/exploration.md)。

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

默认启用真实动态障碍 EXP01，headless，持续打印进度，结束写 final-result.json。Ctrl+C 请求锁存停车并清理本次进程，等待脚本返回；DDS 退出后本机 TCP 回收可能额外等待约65秒。

编辑 `src/t1_navigation/config/exploration_task.yaml` 中的 x/y，即 B 在探索开始时 A 身体系中的米制偏移。A 的 +x 向前、+y 向左；输入不是固定 map 绝对坐标，不指定终点 yaw。运行开始会保存任务快照，中途改文件仅影响下次。原 `presentation/goal.yaml` 属于旧 guided 演示，本入口不读取它。

```bash
./scripts/run_exploration_demo.sh --preflight
./scripts/run_exploration_demo.sh --static  # 无动态实体的对照
./scripts/run_exploration_demo.sh --task /absolute/path/task.yaml
./scripts/run_exploration_demo.sh --static --scene /home/haoyang/humanoid_nav_ws_advanced/src/t1_mujoco/worlds/exploration_south.yaml
./scripts/run_exploration_demo.sh --static --task src/t1_navigation/config/exploration_unreachable_task.yaml --time-limit 20
```

最后一个是预先登记的失败用例：B 在测试世界之外，20仿真秒任务预算后应失败并安全停车，不能统计为成功到达。场景参数只进模拟器，不传给探索进程。

新环境不在本步范围。已有 overlay 若需重建：设置既有 `T1_VENV`、`T1_UPSTREAM` 后 `source scripts/env.sh`，用系统 `/usr/bin/colcon build --symlink-install --base-paths src`；运行入口明确使用配置 venv Python，不重装依赖。

以下为扩展前的静态验收历史，不能代替本轮动态验收；完整动态结果另列。历史正例使用同一任务文件和相同在线实现 `c72179061a08416a51ca17ff5e49c0725d811502`：

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 四包系统构建 / 相关CPU测试 | PASS / 149 PASS | raw/build-02.log、raw/cpu-tests-03.log |
| 北通道未知场景 | SUCCESS，误差0.0915m，轨迹最高y=8.364m | [结果](north.json)、[真实地图与轨迹](north.png) |
| 南通道未知场景，相同B输入 | SUCCESS，误差0.0835m，轨迹最低y=−8.077m | [结果](south.json)、[真实地图与轨迹](south.png) |
| 每次选择时路径已知可达 | 8+8条逐地图快照离线重验PASS | 两个结果的path_replay |

两次正例均有1次从探索直接切换B、零环境/障碍/非足接触与跌倒、真实停车、单一TF和命令ownership、所有组件退出0及端口回收。先前失败：首轮节点字段冲突；第二轮物理到达但graph快照取证竞态；南场景首轮lifecycle启动超时。三次原FAILED分别保留在attempt-01.json、attempt-02.json和south-startup-01.json，不称为无失败可靠性验证。

负例的预登记条件在negative-case.json，最终独立核验写入negative-verdict.json；任务FAILED与负例检查PASS是两个不同结论。最终完整审计见final-verification.json。

原始 run_data/exploration/每个ID包含决策日志、在线地图快照、ROS日志、manifest及最终结果；物理trace在 run_data/同ID。失败保留，运行通过和图形验证分开；W16未运行GUI/录像。raw-index.json保存本步原始产物路径与哈希。

详见 [探索架构](../../architecture/exploration.md)。

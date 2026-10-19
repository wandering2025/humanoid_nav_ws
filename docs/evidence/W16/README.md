# W16 运行与验收

状态以 [W16 执行记录](../../plans/t1_mujoco_navigation/records/W16.md) 为准。

2026-10-20（Europe/Berlin）：用户已测试图形演示并反馈“看起来很棒”，明确要求更新presentation/；现已将一行启动、相对B配置、中英讲稿和验证边界写入 [presentation](../../../presentation/README.md)。这是用户反馈后的文档交付，不新增运行测量；历史审计JSON中的待反馈字段保留其当时含义。

在 Ubuntu Bash 中，从 `/home/haoyang/humanoid_nav_ws_advanced` 执行：

```bash
./scripts/run_exploration_demo.sh
```

Windows PowerShell 中使用：

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws_advanced --exec bash ./scripts/run_exploration_demo.sh
```

默认启用真实动态障碍 EXP01，并同时打开MuJoCo与RViz；持续打印进度，结束写 final-result.json。到达后窗口保留8秒；加 `--hold` 则保持到Ctrl+C。默认图形入口已完成完整动态闭环验证，状态以W16记录为准。Ctrl+C 请求锁存停车并清理本次进程，等待脚本返回；DDS 退出后本机 TCP 回收可能额外等待约65秒。

编辑 `src/t1_navigation/config/exploration_task.yaml` 中的 x/y，即 B 在探索开始时 A 身体系中的米制偏移。A 的 +x 向前、+y 向左；输入不是固定 map 绝对坐标，不指定终点 yaw。运行开始会保存任务快照，中途改文件仅影响下次。原 `presentation/goal.yaml` 属于旧 guided 演示，本入口不读取它。

```bash
./scripts/run_exploration_demo.sh --headless  # 无图形CPU验证
./scripts/run_exploration_demo.sh --no-rviz  # 只显示MuJoCo
./scripts/run_exploration_demo.sh --hold  # 到达后保持窗口
./scripts/run_exploration_demo.sh --preflight
./scripts/run_exploration_demo.sh --static  # 无动态实体的对照
./scripts/run_exploration_demo.sh --task /absolute/path/task.yaml
./scripts/run_exploration_demo.sh --static --scene /home/haoyang/humanoid_nav_ws_advanced/src/t1_mujoco/worlds/exploration_south.yaml
./scripts/run_exploration_demo.sh --static --task src/t1_navigation/config/exploration_unreachable_task.yaml --time-limit 20
```

最后一个是预先登记的失败用例：B 在测试世界之外，20仿真秒任务预算后应失败并安全停车，不能统计为成功到达。场景参数只进模拟器，不传给探索进程。

新环境不在本步范围。已有 overlay 若需重建：设置既有 `T1_VENV`、`T1_UPSTREAM` 后 `source scripts/env.sh`，用系统 `/usr/bin/colcon build --symlink-install --base-paths src`；运行入口明确使用配置 venv Python，不重装依赖。

## 本次图形交付

默认MuJoCo + RViz，一行入口与上方相同，EXP01真实动态障碍仍启用。MuJoCo显示实体与A/B，RViz显示在线SLAM、原始雷达、实际Nav2路径、当前探索目标及实际走过轨迹。A/B采用探索器锁存的实际坐标。仅知A系B输入、无预设观测航点；图形中的完整仿真场景不提供给探索器。

当前在线实现 `7fa9eeb1daed42cb74cd8067f547c46b4c643920`（完整版本以W16记录与git为准）。完整GUI run `20261019T220018Z_exploration_f55dc7ff-76c0-4473-af02-96b4fc3657a1` **SUCCESS**；7条选择路径全部按当时观测快照独立复验。动态实体6.8m、活动scan83帧/costmap37帧，真实站稳等待17.72s，离开后恢复4.132m，到B最后2s最大误差0.095193m。零接触/跌倒；graph/TF/停车、全部进程退出0、无残留与DDS释放全部通过。

图形审计17项全true：NVIDIA renderer、A/B显示坐标与实际目标一致、六张实际窗口截图、GPU/宿主监控无故障；温度最高65°C、显存2159MiB、负载64%。mapping输出最大年龄0.06s。最终真实SLAM旧阻塞footprint649个采样点全自由、无占据/未知。构建四包PASS，171 CPU tests PASS。

同代码 `--headless --static` 对照 **SUCCESS**，末段最大误差0.062906m、8条观测路径全通过；模型nmocap=0。安全/graph/退出/无残留/端口全部PASS。`gui-final-verification.json` 15项全true，动态+对照共15条路径，所有src/scripts字节相同；`gui-raw-index.json` 为本轮5次运行与原始日志的389文件索引。该审计生成时用户反馈尚待收；后续反馈与presentation交付见上方日期说明。

本轮3次开发失败完整保留：GUI负载下mapping积压/TF外推、原始scan墙钟间隔超过0.3s触发停车、局部等待预算在costmap仍阻塞时耗尽。修复含过滤计算优化/队列限制、15Hz显示刷新、有限等待后从已知安全区换位观察；所有安全与物理时限不变。成功GUI轮使用Nav2等待后继续，新的known_free_reobservation分支本轮未触发，其CPU验证不算单独物理验证。`arrived-rviz`截图保留了上一渲染帧的Waiting状态文字；到达结论依据真实轨迹/站稳和运行结果，不依据文字。

证据：gui.json/png、gui-verification.json、gui-dynamic-audit.json/png、gui-{ready,dynamic,arrived}-{mujoco,rviz}.png、gui-attempt-01/02/03.json。原始run_data和构建日志均ignored。不要把少量成功当作任意负载/布局的可靠性保证；原W13 FAILED及历史启动偶发超时保持。

图形交付时按用户要求保留presentation/，待用户测试反馈；该条件已于2026-10-20满足，新增命令见 [演示启动说明](../../../presentation/COMMANDS.md)。

此前headless动态实现 `491d2c837e68204393bcaa129e04619fceadb2ba` 的D1–D3已通过；下表是该历史版本，不能替代当前GUI版本验收。

| 动态检查 | 实测结果 |
| --- | --- |
| 物理实体运动 | 3×0.6×1.5m MuJoCo碰撞体，累计6.8m，轨迹完成 |
| 原始雷达 / 实时costmap | 活动期231命中帧 / 36标记帧 |
| 真实响应 | 站稳等待15.1s，事件结束后行走4.142m |
| 到达 | 最后2s最大误差0.1081m，站稳≥2s |
| 持续SLAM | 无暂停；8916/9846个活动障碍命中从建图分支排除，原始scan不变 |
| 阻塞解除 | 最终实际SLAM图中原阻塞footprint649点全部自由，0占据/未知 |
| 运动与回收 | 零接触/跌倒，TF与命令唯一，全部进程退出0/端口释放 |

[动态结果与逐路径审计](dynamic.json) · [真实地图与轨迹](dynamic.png) · [动态感知/等待/清空审计](dynamic-audit.json) · [动态响应时间图](dynamic-audit.png)。

同代码无障碍对照使用 `--static`，误差0.06757m，7次决策全部逐图复验通过，零接触/跌倒、正常停车与完整清理。[对照结果](dynamic-static-control.json) · [轨迹](dynamic-static-control.png)。

[最终14项审计](dynamic-final-verification.json)全部PASS；两轮14个在线源码/参数文件一致，任务快照相同，9+7条实际选择路径全部来自已观测可达区。[原始数据索引](dynamic-raw-index.json)包含本轮5次运行的291个文件，失败保持FAILED。

本轮失败完整保留：回调签名不兼容、静态墙误分类后主动停止、等待取消触发安全锁存后提前退出，分别见dynamic-attempt-01/02/03.json及dynamic-failed-finalization.json；后续修复并重新运行，未把这些失败改写成成功。158项CPU测试、四包build通过；不是批量可靠性评测，原W13 FAILED保持。

该历史版本使用理想odom、有限束补偿LiDAR和不可被推动的mocap运动学障碍，当时仅验证headless仿真；后续GUI结果见上方。过滤器不能保证识别最初静止于未知区域的物体，不宣称通用动态SLAM或真机能力。

以下为扩展前的静态验收历史，不能代替本轮动态验收；完整动态结果另列。历史正例使用同一任务文件和相同在线实现 `c72179061a08416a51ca17ff5e49c0725d811502`：

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 四包系统构建 / 相关CPU测试 | PASS / 149 PASS | raw/build-02.log、raw/cpu-tests-03.log |
| 北通道未知场景 | SUCCESS，误差0.0915m，轨迹最高y=8.364m | [结果](north.json)、[真实地图与轨迹](north.png) |
| 南通道未知场景，相同B输入 | SUCCESS，误差0.0835m，轨迹最低y=−8.077m | [结果](south.json)、[真实地图与轨迹](south.png) |
| 每次选择时路径已知可达 | 8+8条逐地图快照离线重验PASS | 两个结果的path_replay |

两次正例均有1次从探索直接切换B、零环境/障碍/非足接触与跌倒、真实停车、单一TF和命令ownership、所有组件退出0及端口回收。先前失败：首轮节点字段冲突；第二轮物理到达但graph快照取证竞态；南场景首轮lifecycle启动超时。三次原FAILED分别保留在attempt-01.json、attempt-02.json和south-startup-01.json，不称为无失败可靠性验证。

负例的预登记条件在negative-case.json，最终独立核验写入negative-verdict.json；任务FAILED与负例检查PASS是两个不同结论。最终完整审计见final-verification.json。

原始 run_data/exploration/每个ID包含决策日志、在线地图快照、ROS日志、manifest及最终结果；物理trace在 run_data/同ID。失败保留，运行通过和图形验证分开；以上早期静态阶段未运行GUI/录像，后续GUI验证见上方，W16仍未录制视频。raw-index.json保存早期静态阶段原始产物路径与哈希。

详见 [探索架构](../../architecture/exploration.md)。

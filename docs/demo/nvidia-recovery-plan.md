# W14 NVIDIA分阶段恢复预登记

2026-10-15。用户明确授权使用RTX4050并自行分阶段选择较低风险参数；禁止安装/卸载/升级/回滚/重启驱动，
禁止修改BIOS、注册表、显卡切换模式和系统显示设置。
用户后续更新边界：允许逐个增加已验证GPU任务的并发，按显存、温度、响应和日志自主调整；
异常时停止扩载、终止最近GPU任务、保存结果，不自动重试同一路径。监控不构成故障隔离保证。
用户另行明确授权：完整保留CPU辅助FAIL，另行预登记两次同seed、同导航参数的默认GUI正式排练。
本记录不改W13 FAILED、不改CPU结果、不放宽动态停车0.5s或任何安全/导航/物理判据。

起始HEAD `89628a2bca0726a1131349a999f73f81f141e1e8`，工作区clean，发布分支不变。
初始只读NVML：RTX4050、驱动596.58、47°C、637/6141MiB、21%负载（不是稳定性证明）。

分阶段，前阶段全部正常退出后才允许后阶段：
1. 明确MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA GeForce RTX 4050，最小glxinfo上下文。
2. 单个320×240 OpenGL窗口，10fps，15s，确认实际GL_RENDERER含NVIDIA/4050、不含Intel。
3. 单个官方T1静态模型窗口，800×600，15fps，30s，仅绘制，不推进物理/策略，不冒充导航。
4. 通过后才开放一行默认GUI入口：先启动MuJoCo NVIDIA，真实就绪后再启动CPU软件RViz，避免两个GPU渲染器。
   准备30s单窗口MuJoCo站立/安全退出检查；不发导航目标、不修改步态参数。
5. 固定U01/4000默认正式排练01/02，命令均为./scripts/run_demo.sh；不重跑替换任一次失败。

仅为本次进程设置GPU适配器/线程/窗口显示参数，不修改全局设置。MuJoCo本机OpenGL在WSLg下仍经Mesa/D3D12，
这里避开Intel适配器，不声称绕过全部Windows显示驱动。CUDA无需用于原本CPU的官方策略，避免改变策略数值路径。

监控：每秒有界NVML查询，采集temperature/memory/utilization/power；每次查询超时即停止，不能用N/A冒充正常。
保守停止条件：温度≥75°C；显存≥总量70%或比本阶段起始增加≥768MiB；负载≥95%持续5s；
D3D12 Removing Device、GPU/Xid/device lost错误、渲染心跳或真实clock停滞、窗口关闭/异常退出。
这些是演示运行资源停止线，不是硬件安全保证，也不修改机器人安全区。
只关闭本次登记PID；异常写入persistent halt文件，后续阶段不得自动继续。监控不能保证在宿主驱动完全失去响应前恢复桌面。

图形窗口大小/帧率只在canary中逐级增加；正式仿真viewer_rate_hz沿原冻结30Hz，不偷偷改物理/导航参数。
默认RViz采用软件渲染并保留设备拒绝规则，启动串行；双视图必须如实披露渲染方式。
录像仍缺失，工具不具备可靠完整双视图录制时按原规则保留W15前人工动作。


实际执行结果：阶段1检查器标题解析误报已离线修正、原FAIL保留；阶段2/3通过。阶段4两次站立检查出现CPU RViz GLSL链接错误且退出端口FIN_WAIT检查失败；发现shader错误后停止GPU，profile锁定。阶段5两次默认GUI正式排练均未开始。最后shader检测和进程退出后端口核验只做非图形单元检查，不继续GPU重试。详见docs/evidence/W14/nvidia-recovery/README.md。

## 更新边界后的替代显示路径预登记

起始HEAD `8aa6ab77cfc56bbf708a6c456feed83f734740c8`；已有未提交W14文档/证据和RGB显示修复保留。
禁止再次加载原MapDisplay indexed_8bit shader。只订阅原OccupancyGrid，生成RGB PointCloud2供显示；
不发布map/costmap/TF/cmd，不改变导航输入、物理或阈值。移除RViz文字marker，保留安全圆和目标。
受Landlock保护CPU回放 `20261015T130147Z_cpu-rgb-cells`：20s显示无GLSL错误、RViz0；
DDS退出−6（Bad file descriptor），整体退出未通过，原result标PASS的脚本缺陷由独立assessment纠正，不覆盖原结果。
原GPU halt完整归档；只为这个已改变且CPU验证过的显示组合开放一次无导航站立检查。
任一图形错误立即停止，不重试。仅该检查全通过才开放默认GUI正式01/02，固定U01/4000，
两次命令都为 `./scripts/run_demo.sh`，同一干净源码SHA。任何导航失败保留并判W14_BLOCKED，不换seed。
正式01/02不使用 --startup-check/--headless/--no-rviz；每次独立新PTY，不继承ROS/venv，仍保留原PYTHONPATH。
记录图形/安全/动态/退出/端口指标。允许并发不要求并发；当前继续单GPU MuJoCo+CPU RViz以减少负载。

替代组合站立check已通过：run 20261015T130850Z_demo_U01_8243f83a-c28e-47af-a8af-963adebac875，
被测a99a29752545982fd9c42813a2cc8d54f752db9e；final SUCCESS_STARTUP_CHECK，全部进程0、端口可绑定、无shader错误。
profile晋级GUI_VALIDATED只表示分阶段入口条件满足，不表示两次正式排练或W14已通过。

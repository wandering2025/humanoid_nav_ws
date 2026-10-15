# W14 CPU隔离续作：仍为W14_BLOCKED

起始clean HEAD `2f665a561ae53be63e3b660eae33eec287b4060b`；无委派，无GPU/GUI重试。
隔离实现 `577484775fc0a64ceeee87020a5d388a640a4ade`；CPU入口 `9506fb6afd5088a56459f51288b3b5d320ddc5a1`；
完整物理被测HEAD `e4be3477e78015f57ce01c847f204bb2caac3546`（clean）。

唯一辅助run `20261015T122606Z_demo_U01_1ddbd7a9-bd24-4742-a38f-a9513561e809`，
命令 `./scripts/run_demo.sh --headless`。它不是原两次默认GUI排练，也不进入W13分母。

Nav2和独立truth到达：0.161070174m、6.194888078°，零碰撞/跌倒，10Hz全身采样净距1.764366959m。
就绪15.244811114s；到达后观测结束92.905991610s；正常退出记录总94.292266397s（不含随后离线分析）。
真实LiDAR命中、costmap标记/清除、重规划、实际停步后恢复均有记录；
但**保护区内连续满足冻结静止条件0.42s，小于原0.5s**，dynamic verdict FAIL。
原result FAILED完整保留；没有放宽阈值、换seed或重跑物理。后续只修打包层TCP诊断。

安全stop服务ACK且按0.5s速度窗连续2s站稳；DDS/launch/goal均exit0，17个launch组件均exit0。
无SIGTERM/SIGKILL升级、无人工干预、登记PID无残留。立即无LISTEN，但bind受内核TIME_WAIT影响而失败；
自然到期后只读复查无TIME_WAIT、端口可bind。原失败字段未改，post-run-audit单列后续事实。
当前代码新增有界等待TIME_WAIT逻辑，9编排+3隔离测试PASS；该最后修复没有再跑物理来刷成功。

Landlock ABI7在runtime导入前限制read/write/ioctl；用无害/dev/full验证拒绝，未为验证而打开GPU。
GPU显示/CUDA可见设备清空仅限本次子进程，PYTHONPATH未清空。19个当时存活登记进程抽查均NoNewPrivs1且无GPU FD。
隔离只覆盖本次Linux进程，不是整机零风险保证；默认图形入口继续封锁。

旧overlay 3个生成物逐路径归档至ignored run_data/demo-overlay-archive-20261015T1225Z，4包重建2.99s成功。
没有删除/覆盖实验分支源码、历史证据或上游，导航/控制/物理/安全阈值无变化。
原始trace/geometry/log保持ignored，raw-artifact-index记录全部路径/大小/hash；本目录拷贝的result与原字节相同。

# 演示发布清单

最终状态：**W14_BLOCKED**。不启动W15，不推荐当前版本用于现场演示。
原目标入口 `./scripts/run_demo.sh` 默认GUI仍因宿主事故封锁，返回3。
--preflight/--headless仅在内核CPU隔离下开放，无GPU设备访问/窗口/CUDA探测；不代表GUI恢复。
[安全审查](graphics-incident-20261015.md)；[机器可读证据](../evidence/W14/incident-summary.json)。

| 项目 | 完整版本 |
|---|---|
| 发布分支 | release/w13-formal-v1-demo |
| 发布基线 | b23a92465b317ca781d3eb210051126c5786908e |
| 首次W13实现/证据 | 04d74f17a6d56d24d7da5ae8ff0c9a4398608daf |
| DEMO_DECISION_SHA | ebbf062da8db33c691935a64ab327fbfccbd122f |
| 演示入口/RViz实现 | dbb06b8dacbab48de82f8982fbe7a8ebf27da15a |
| GUI事故被测HEAD（clean） | b1ab66fecace5aff3acf4ed6fbefa4cfb85ac3af |
| 安全封锁/静态清理防护 | 85192d0d4c45bb8cdeaffe4c42e643c0894b412e |
| 保留实验分支HEAD | 7b58acd77de722c9b53f4b69271dde862d7f0ff3 |

首次W13仍FAILED，用户接受状态DEMO_BASELINE_ACCEPTED_WITH_KNOWN_LIMITATIONS。
本分支原导航/控制/安全/物理/事件文件与基线无差异；只新增demo.rviz。
正式40例来源仍在formal-v1逐run JSON/summary，W13原始记录未覆盖，后续实验历史完整保留。

| 排练 | 命令/ID | 结果 |
|---|---|---|
| 1 | `./scripts/run_demo.sh`；20261015T114444Z_demo_U01_6c4ff40d-5119-4308-8252-a71715a2a1b7 | FAIL_GRAPHICS_HOST_INCIDENT；未ready、未发目标、未到达 |
| 2 | 预定同一默认命令；无run ID | NOT_RUN_SAFETY_BLOCK，不伪造第二次成功 |

固定U01/seed4000，A(0,0,0)→B(9,0,0)，原w13_events_v1.yaml不改。
启动就绪耗时、完整导航/总耗时、最终位置/yaw误差、全程碰撞/跌倒/最小净距均 **N/A**，
没有完成run可供核验。持久化trace到sim5.50s，275行策略输入全0；不以部分无异常推断全程安全。
可恢复退出整数：initial_pose1、starter1、localization manager−9；其余逐项见incident-summary。
MuJoCo/RViz曾升级到SIGKILL，最终退出整数和DDS/launch父退出整数缺失，不能填0。
需要Ctrl+C和用户强制重启；脚本安全停车/回收验收FAIL。
重启后宿主只读审计：本项目进程无，TCP42123/42124/42126/42127无监听；不是脚本清理通过。

录像：**MISSING**；路径/大小/SHA256均N/A，可播放验证NOT_RUN。
[人工录像步骤](troubleshooting.md)仅在事故恢复和重验后执行，W15冻结前必须补齐。
本次没有安装录屏工具或提交视频/bag/完整原始运行数据。

已知限制：理想odom/姿态补偿、有限束/无噪声、平地宽通道、mocap无动力响应、无动态预测、
低速死区与停车回弹、非确定调度、W13四失败及严格headless RTF失败；新增双显卡TDR与未通过的启动/清理。
详见[架构限制](architecture-and-limitations.md)。

W15开始前首先读 `docs/plans/t1_mujoco_navigation/交接记录.md`、`records/W14.md`、本文和事故审查。
当前不满足W15前置；必须先完成事故恢复审查及W14剩余验收。最终交接SHA见最终记录提交（不自引用）。


## CPU安全续作（辅助验证，不替换上表）

- 设备隔离SHA：`577484775fc0a64ceeee87020a5d388a640a4ade`。
- CPU入口SHA：`9506fb6afd5088a56459f51288b3b5d320ddc5a1`；唯一物理被测HEAD `e4be3477e78015f57ce01c847f204bb2caac3546`，clean。
- 最终打包修复SHA：`7d29b700689a870bdc8c57bac7aee74144a55082`；该最后端口修复只做单元检查，未重跑物理。

- run ID：`20261015T122606Z_demo_U01_1ddbd7a9-bd24-4742-a38f-a9513561e809`。
- 命令 `./scripts/run_demo.sh --headless`，U01/4000不变；不是默认GUI正式排练。
- 到达：True；误差0.161070174m/6.194888078°；接触三类均0、跌倒False；采样全身最小净距1.764366959m。
- 就绪15.244811114s；到达后观测结束92.905991610s；退出记录总94.292266397s（不含后续离线分析）。
- LiDAR/局部costmap/重规划/恢复运动均观测；**保护区连续满足静止条件0.42s<原0.5s，dynamic FAIL**。
- 安全stop ACK并按0.5s窗连续2s站稳；DDS/launch/goal与17个launch组件均exit0，无升级信号，无人工干预、无剩余登记PID。
- 立即端口bind受TIME_WAIT影响失败，原result不改；随后自然到期审计无监听/无TIME_WAIT且可bind。
- 无GPU窗口/设备调用；19个当时存活登记进程抽查均NoNewPrivs1且无GPU FD。3隔离+9编排测试PASS。
- CPU结果整体仍FAILED；不降门槛、不换seed、不再跑物理。图形根因未解除，两次默认GUI验收仍不满足。
- 视频仍MISSING，路径/大小/hash/播放验证仍N/A/NOT_RUN。

完整[CPU证据](../evidence/W14/cpu-recovery/README.md)和[机器可读裁定](../evidence/W14/cpu-recovery/assessment.json)。


## 用户授权NVIDIA阶段（当前仍BLOCKED）

用户允许新GUI正式两次排练并保留CPU失败；实际只完成最小GL、320×240、静态T1渲染和两次无目标站立检查。
NVIDIA渲染实测最高61°C、787MiB，未触发温度/显存/驱动超时停止线；Windows事后查询无匹配新Display/141/41事件。
但CPU软件RViz报告GLSL链接错误，监控先前漏检，离线审查发现后立即锁住GPU，不再启动窗口。
当前profile为GPU_HALTED_GRAPHICS_ERROR；原资源停止线未放宽，未改驱动/BIOS/注册表/系统显示。
最后实现 `8aa6ab77cfc56bbf708a6c456feed83f734740c8`：补shader错误检测、独立进程退出后核验端口；25项非图形测试通过，未动态复验。

站立01 `20261015T124608Z_demo_U01_86257d22-2b23-47b6-b746-995b45f71937`：就绪18.7455s，记录50.4997s。
站立02 `20261015T124845Z_demo_U01_264e5d1e-0094-4a1e-ab91-602788592bd1`：就绪18.2261s，记录49.9732s。
均未发导航目标，三类接触0/跌倒False，全部登记进程exit0；原FIN_WAIT端口失败保留。事后无残留且端口可绑定。
新GUI正式01/02均NOT_RUN，无run ID；不能冒充已完成两次默认排练。录像仍MISSING。
见[完整证据](../evidence/W14/nvidia-recovery/README.md)。

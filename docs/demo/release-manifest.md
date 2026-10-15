# 演示发布清单

最终状态：**W14_BLOCKED**。不启动W15，不推荐当前版本用于现场演示。
原目标入口 `./scripts/run_demo.sh` 已实现但因宿主黑屏事故硬封锁，当前返回3；只有--help可用。
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

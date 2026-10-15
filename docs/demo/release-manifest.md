# W14 演示发布清单

最终状态：**W14_BLOCKED**。本次W14未完成；W15未启动。GUI在任何GPU监控/上下文创建前拒绝（exit3）。
直接阻塞：最新完整默认运行已到达、dynamic PASS，但CPU RViz在SIGINT退出时报告 `corrupted double-linked list`、exit−6。
原失败不改；没有自动重试此路径，第二轮NOT_RUN。设备监控不是故障隔离或整机安全保证。

| 项目 | 完整SHA |
|---|---|
| 发布分支 | release/w13-formal-v1-demo |
| 发布基线 | b23a92465b317ca781d3eb210051126c5786908e |
| 首次W13实现/证据 | 04d74f17a6d56d24d7da5ae8ff0c9a4398608daf |
| DEMO_DECISION_SHA | ebbf062da8db33c691935a64ab327fbfccbd122f |
| 原入口/RViz实现 | dbb06b8dacbab48de82f8982fbe7a8ebf27da15a |
| RGB替代显示与站立证据 | c22f36e68f5bc3a5339329c369568628de4610c3 |
| DDS等待修复 | 5fb2faa0310bb2fa0e5786aaa0e09b9ea00546c7 |
| 初始零clock修复 | bbef78f5a3cbed60c79b2a4f0d59fb838b1d8c8a |
| 最终实现/停止保护 | 457dc8f01ffac88582e6017c631dd4a6f2d9f3a6 |
| 保留实验分支HEAD | 7b58acd77de722c9b53f4b69271dde862d7f0ff3 |

W14最终交接SHA由后续最终记录提交的 `git log -1`读取，不自引用；权威交接为
`docs/plans/t1_mujoco_navigation/交接记录.md`及`records/W14.md`。
未reset/rebase/amend/删除分支/push。原src仅新增demo.rviz，导航/控制/安全/物理/地图/事件逐字匹配基线；
W13结果/证据/记录保持不变。formal-v1完整40例来源见[W13证据](../evidence/W13/formal-v1/summary.json)。

首次W13仍FAILED：static10/10、crossing8/10、blocking8/10、sudden10/10，总36/40；环境碰撞/跌倒0；
严格headless RTF0.9999998608211204<1。用户接受状态为DEMO_BASELINE_ACCEPTED_WITH_KNOWN_LIMITATIONS，
不等于通过统计可靠性或安全认证。

## 全部GUI正式尝试（失败均保留）

命令均为 `./scripts/run_demo.sh`，U01/seed4000。每次新的env -i Ubuntu PTY，保留显示与PYTHONPATH、无ROS/venv继承；
未亲手关闭/重开Windows Terminal界面。每批预登记同一干净SHA，打包修复后另立批次，不替换旧结果。

| 批次/轮 | run ID | 被测SHA | 实际结果 |
|---|---|---|---|
| 原事故01 | 20261015T114444Z_demo_U01_6c4ff40d-5119-4308-8252-a71715a2a1b7 | b1ab66fecace5aff3acf4ed6fbefa4cfb85ac3af | Intel DX12相关宿主黑屏；未ready/未发目标；强制重启 |
| 原事故02 | 无 | 未运行 | NOT_RUN |
| 新批1/01 | 20261015T131303Z_demo_U01_3c885c0f-7907-4cfb-8a9b-a5879600cd0b | 23217d6b79332171c2f4ccb91cdf85218559352d | SUCCESS，到达/动态/退出/端口通过 |
| 新批1/02 | 20261015T131632Z_demo_U01_11cf967e-10d7-49b9-bbbf-3b07fcbaac7b | 同上 | FAILED，观察器无首clock，物理trace13.42s；未发目标 |
| 新批2/01 | 20261015T131927Z_demo_U01_21579b69-8303-495e-9611-adc3ccb4b9db | 2a211be8b6f4f94bf5cb4db40e20e9848ab30323 | FAILED，新增观察器把初始零clock误作停滞；未发目标 |
| 新批2/02 | 无 | 未运行 | NOT_RUN，先修复打包边界 |
| 最新批3/01 | 20261015T132133Z_demo_U01_89a56cbe-0723-4ca9-b438-1021c94912e9 | 17aae04e368614f97eb5629a8d855a6c732ec719 | 到达/dynamic PASS，但RViz退出−6，整体FAILED |
| 最新批3/02 | 无 | 未运行 | NOT_RUN_SAFETY_HALT；不能凑成两次通过 |

两次完整到达GUI运行详细指标（来源SHA不同，不能合并成同版本两次验收）：

| 指标 | 新批1/01 | 最新批3/01 |
|---|---:|---:|
| 独立位置误差m / yaw° | 0.157421787 / 0.090666825 | 0.119849749 / 5.903550108 |
| 接触static/obstacle/nonfoot / 跌倒 | 0/0/0 / False | 0/0/0 / False |
| 全身10Hz采样最小净距m | 1.523471426 | 1.715809258 |
| 动态保护区连续静止s | 7.12（原≥0.5） | 1.24（原≥0.5） |
| LiDAR命中帧 / 标记 / 清除样本 | 776 / 123 / 106 | 811 / 132 / 115 |
| 组件启动就绪s（不含入口预检） | 18.5404 | 18.0399 |
| 到达后观测结束wall s（从编排开始） | 94.0862 | 93.5659 |
| 原任务sim s | 60.8 | 60.8 |
| 退出记录wall s（含保留画面/清理） | 110.3839 | 115.7033 |
| 另加runner退出后TCP冷却s | 60.1963 | 60.0530 |
| 命令至ready / 至最终文件约s | 29.02 / 196.44 | 29.80 / 198.80 |
| DDS / launch / RViz / goal退出 | 0 / 0 / 0 / 0 | 0 / 0 / −6 / 0 |
| launch管理的17组件 | 全部0 | 全部0（含MuJoCo） |
| 安全stop ACK / 站稳 | True / True | True / True |
| 人工干预 | 无导航干预，仅最终画面Ctrl+C | 同左 |
| 剩余PID / 端口42127 | 无 / 可bind | 无 / 可bind |
| GPU最高°C / MiB / 利用率% | 62 / 1009 / 52 | 62 / 1008 / 50 |

命令全程估算来自秒精度run ID与结果文件落盘时间（含预检、离线分析、TCP冷却；约±1s，系统墙钟调整可影响），
其余wall数值为monotonic实测，任务sim时间另列。净距是10Hz样本最小值，不是连续下界；500Hz接触另计。
安全链完整、真值隔离与单次initialpose/无clock回退检查见逐run JSON。原CPU辅助动态0.42s FAIL不改。

## 图形审查与当前停止边界

遵守用户更新边界：优先明确NVIDIA，最小负载→小窗→静态T1→站立→完整任务逐步验证；
允许并发但本次保持单GPU MuJoCo+CPU RViz。无驱动/BIOS/注册表/显卡模式/系统显示更改，无依赖升级。
MuJoCo OpenGL在WSLg仍走Mesa/D3D12 NVIDIA后端，不声称原生Linux直连或故障隔离；官方策略沿原CPU路径。
CPU RViz由Landlock拒绝GPU设备，以llvmpipe绘制RGB网格，原MapDisplay shader路径禁用。
GLSL历史漏检已披露；本次堆损坏在退出审查发现，不能声称实时监控已捕捉。新检测只做单元验证。

本次直接证据是RViz原生堆损坏，不能凭温度正常排除驱动或确定具体库/线程根因。
事后进程审计无本项目残留，42127/42128可bind；13:00UTC后Windows事件查询未匹配Display/141/41，无查询错误。
无新宿主事件不等于绝对安全。当前profile GPU_HALTED_RVIZ_HEAP_CORRUPTION，halt保留，不自动再试。

## 录像及剩余动作

录像 **MISSING**：路径/大小/SHA256均N/A，播放验证NOT_RUN。本环境无可靠完整双视图录制工具；未安装软件。
[人工录制精确步骤](troubleshooting.md)是W15冻结前必须完成的明确动作，须先解除当前W14阻塞并复验。
不把截图、静态canary或失败片段冒充成功录像。

尚缺同一干净源码两次完整默认排练全部到达且安全退出；RViz异常退出须定位并验证，禁止刷重跑冲销失败。
已知限制：[架构限制](architecture-and-limitations.md)。W15开始必读权威交接、W14记录、本文、事故审查及
[全部排练证据](../evidence/W14/rehearsals/README.md)。当前不满足W15前置。

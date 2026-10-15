# 2026-10-16 优化验收

**ADVANCED_DEMO_BLOCKED_STABLE_V0_1_PRESERVED**。功能性无界面演示通过；新图形展示与新录像未验收，等待GPU受控恢复授权。两项用户反馈已落实为可测量的运动和路径显示改动，不能把这份结果写成新的图形发布。

| 项目 | 实测/状态 |
|---|---|
| 完整动态运行 | `20261015T234553Z_advanced_12517517-2132-4dd2-a997-3dc793ec7747`，SUCCESS |
| 完整运行源码SHA | `2875228d3c27e293c80087453543be0c549f3347` |
| 最终实现SHA | `2875228d3c27e293c80087453543be0c549f3347`；与完整动态运行及对照的代码一致 |
| 证据SHA | `0b9ba65b30585d07d99db0f18cd70cff5ba8deae` |
| 同版本无障碍对照 | `20261015T234208Z_advanced_89cb184c-a79b-4e26-b251-29ccbbe46d2b`，SUCCESS |
| 真实等待 | 16.10s，撤离后继续7.22m |
| 扫描/costmap | 188帧 / 32帧 |
| 实际地图 | 已知面积2.85→329.37m²；最终西区100.00%、北廊99.98%、完整东区95.63% |
| 路径显示数据 | 同时捕获2次Nav2与显示消息SHA256一致；真实等待状态/路径年龄更新已观察；未验证图形渲染 |
| 四项安全结果 | 静态0、动态0、非足接地0、跌倒0 |
| 采样机体最近间距 | 2.200m，10Hz；非连续下界 |
| 到达与停车 | 原工具验证成功；锁存停车、物理静止、SLAM关闭、全部退出0、无残留、端口释放通过 |
| 最终接近 | 16.28sim s，终端重试0次 |
| 末端轨迹效率 | 动态/对照路程与端点直线距离比1.144/1.073；航向跨度1.206/0.681rad，无末端绕圈 |
| 轨迹距离 | 旧版76.71m → 本轮30.15m，0.5s采样 |
| 流程仿真时长 | 旧版374.66s → 本轮169.40s；不同任务流程/展示配置，不是算法基准 |
| A-B地图概览 | 实际NavFn路径20.28m，经过北廊形成明显转弯 |
| 跟踪控制器 | 高级专用Regulated Pure Pursuit，原RotationShim/适配器/安全链；默认MPPI不变 |
| CPU/构建 | 194PASS，1条仅适用于v0.1源码形状的断言排除；4包构建通过 |
| 中断负测 | [interrupt.json](../evidence/advanced-optimization/interrupt.json)：任务应FAILED，停车和清理通过 |
| 新RViz运行/录像 | NOT_RUN，GPU告警锁尚未解除；无新录像时长或SHA256 |
| 发布tag | 无新tag；`demo-v0.1.0`及`demo-v0.2.0`原对象和目标均未移动 |

地图来自真实SLAM，未注入场景先验。[地图增长图](../evidence/advanced-optimization/map-growth.png)显示主要结构；停放障碍后方仍存在未知区，东墙局部缺口的离线采样最大距离为1.95m，不能称为完整扫描了每面墙。主通道覆盖门槛与完整东区覆盖分开报告。

对照中接近阶段耗时17.44s，动态运行37.54s。物理步态与ROS时序有波动，不能只用总耗时推断干扰；实际停等、命令、扫描、costmap与恢复才是主要证据。全部失败尝试见[attempts.json](../evidence/advanced-optimization/attempts.json)，原始数据绑定见[raw-index.json](../evidence/advanced-optimization/raw-index.json)。

[轨迹对比图](../evidence/advanced-optimization/trajectory-comparison.png)来自真实运行数据，非RViz截图、非录像替代品。图中障碍仅画停留位置；机器人经过该位置前它已撤离。[源码/稳定版审计](../evidence/advanced-optimization/source-audit.json)确认原工作区clean、原入口和保护文件相同。未推送，未改外部仓库、驱动或显示设置；未安装/升级依赖。

GPU阻塞来自此前运行`20261015T220015Z_advanced_c171be81-40f2-4a13-a537-2aa174bce3f5`：显存1126→1895MiB，触发原768MiB增长阈值。峰值61°C，记录中未见驱动错误，但不能据此断定无风险。`run_data/demo-gpu-halt.json`原字节保留；[gpu_watch.py:61](../../scripts/demo/gpu_watch.py#L61) 明确写有“do not retry automatically”，因此保留启动锁。已请求一次受控恢复：保留/归档原锁和全部故障证据，原阈值下先`--startup-check`，再完整图形运行与独立录像。尚未收到授权，不启动新GPU渲染器。

已发布旧版备份录像仍是`docs/evidence/advanced/raw/final-recording-02/advanced-demo.mp4`，368.25s，SHA256`3f9547f14e7ecadd4d0f115c4a562291dc4530e4de7a95d7633041d0f7a3716a`。它属于旧tag，不验证本轮优化。当前推荐无界面命令见[quick-start.md](quick-start.md)；稳定兜底入口保留。

本轮v0.1回归范围：原工作区HEAD、tag、入口和受保护源码的不可变审计通过；未重跑v0.1仿真。此前完整U01成功记录仍保留。最终两个工作区的clean状态在文档提交后复查；运行中断和正常退出均无残留PID，42127可重新绑定。

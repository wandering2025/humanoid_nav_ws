> **W15 当前状态：FINAL_RELEASE_BLOCKED（2026-10-15）。** 新最小 MuJoCo 检查退出 −11，内核记录 NVIDIA `libnvwgf2umx.so` 故障；GUI 已在 GPU 访问前封锁。独立 B01 也发生 120s 超时。没有成功的最终录像，不可将本工程声明为现场发布完成。详见 [最终审查](../results/final-review.md)。以下 W14 内容保留为历史或讲解草案，不代表当前启动许可。

# 求职讲解提纲

当前W14_DEMO_READY_WITH_DDS_TEARDOWN_WARNING；求职演示发布，含明确关闭警告，不是稳定性认证。

- 目标：官方策略+接触动力学的T1，以真实仿真LiDAR感知动态障碍，自主A→B。
- 展示：MuJoCo步行/实体；RViz地图、scan、AMCL、全局路径、局部costmap与安全状态。
- 架构：四包；AMCL+NavFn/MPPI；命令整形在安全过滤前；唯一clock/TF/命令owner。
- 安全：gate→smoother→CM→只减不增bridge，失联停车；不将软件仿真称为认证。
- 证据：W13正式36/40 FAILED，四失败保留；固定U01是演示，不是新随机统计。
- 限制：理想odom/姿态补偿、有限束、平地宽通道、mocap、无预测、步态死区/回弹。
- 工程限制：历史黑屏、CPU失败、RViz−6及DDS−11均保留。新RViz最小与两次完整均0；两次均到达并安全停车。
- 第二轮原FAILED保持；发布解释为NAVIGATION_SUCCESS / SAFETY_PASS / DDS_TEARDOWN_WARNING。DDS−11只发生在预期关闭后，其他组件全0、无强杀或残留，按14项门槛接受警告；不声称外部DDS已修复。
- 录像FINAL_VIDEO_PENDING_W15；本次没有新增GPU/GUI运行，未启动W15。

“为什么不是100%？”：启动/通信和末端收敛仍有失败。用户接受演示基线不改变统计结论，
不应以精选场景代替可靠性声明。“是否建图？”：图来自先前真实LiDAR/SLAM，默认加载图做AMCL。
“是否真机安全？”：没有真机验证或安全认证。“是否预测动态轨迹？”：没有。

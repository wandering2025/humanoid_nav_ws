```bash
/home/haoyang/humanoid_nav_ws_advanced/scripts/run_advanced_demo.sh --headless
```

当前优化版本已完成真实无界面建图、动态障碍停等/恢复和到达验证。**ADVANCED_DEMO_BLOCKED_STABLE_V0_1_PRESERVED**：此前运行留下的 GPU 告警锁仍在；新 RViz 画面和新录像尚未验收。不要把旧 `demo-v0.2.0` 录像当作本次优化的验证。当前结果与恢复入口见 [优化验收记录](optimization-review.md)。

机器人从 A=(0,0) 沿宽敞西区、北廊和东区连续建图。观测航点只要求通过位置，不逐点精确停稳/校正朝向。第5点起用实际 NavFn 结果检查已观测地图能否规划 A→B；且东区主通道指定区域已有≥95%已知格子才结束这段建图，必要时补第6个向东观测点。**不返回 A，不重复穿越建筑。** 随后从当前位置导航到接近点(12,2.5)，给原工具保留1.5m直线接近段，走向 B=(12,0)，使用原有严格到达/停稳验证。

高级模式使用 NavFn 规划和 Regulated Pure Pursuit 路径跟踪，减少末端绕圈；稳定模式的 MPPI 配置保留。最终动态/对照运行均到达且零终端重试，详细轨迹指标见验收记录。

橙色移动障碍宽3m，以0.6m/s从(15.9,3)进入(12.5,3)，停8秒再退回。Nav2 从 LiDAR/costmap 感知阻塞，等待通路恢复后继续；没有由障碍真值发送停车指令。验收要求至少2秒实际停等、零最终速度命令、真实扫描/costmap证据以及撤离后的继续前进。

RViz 配置和发布已实现（本次尚未图形复验）：蓝色为最新实际 Nav2 执行段路径，标出路径年龄；青色为建图后计算的完整 A→B 路线概览，仅供展示；紫色为实际已走轨迹；橙色为当前目标。地图和红色 LiDAR 保留。目标、轨迹和状态以5Hz刷新，Nav2在新目标、路径失效或原5秒刷新周期重规划，不把显示刷新频率冒充规划频率。

静态建图和动态导航分阶段：地图可用后暂停 SLAM 新测量与位姿修正，保留已建地图；导航定位依赖原有理想里程计和最后的 map→odom 修正，LiDAR 避障持续实时工作。验证到达 B 后恢复 SLAM 并保存最终地图。不是 frontier exploration，也不是全程持续扫描匹配定位。B距南墙4米，Nav2与最终工具均保持原0.20米/0.20弧度容差。

按一次 Ctrl+C 请求锁存停车和清理；等终端结束，不要全局 pkill。TCP TIME_WAIT 可能再等最多65秒。正常入口不录制。GPU 受控恢复获准且最小图形检查通过后，图形命令为同一入口去掉 `--headless`；`--hold` 可延长最终画面。

稳定兜底保持不变：`/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh`。已发布标签 `demo-v0.1.0` 和 `demo-v0.2.0` 均未移动；当前优化提交尚未形成新的图形发布。

[真实轨迹对比](../evidence/advanced-optimization/trajectory-comparison.png) · [架构与限制](architecture.md) · [讲解提纲](presentation-outline.md) · [旧版发布记录](release-manifest.md)

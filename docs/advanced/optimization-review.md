> 本页保留 `demo-v0.2.1` 固定默认B的发布验收。当前工作分支新增可编辑B点的presentation接口，验证见[presentation/VALIDATION.md](../../presentation/VALIDATION.md)；旧标签与录像未移动或覆盖。

# 2026-10-16 优化最终验收

**ADVANCED_DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**。用户反馈的两项问题已通过完整图形运行、真实录像及数据验证。最终版本和视频身份见 [发布清单](optimization-release-manifest.md)。

| 验收项 | 实测结论 |
|---|---|
| 1. v0.1和原入口 | PASS：原工作区HEAD/tag/受保护源码和run_demo.sh不变，clean |
| 2. 独立一行启动 | PASS：run_advanced_demo.sh；图形完整运行SUCCESS |
| 3. 自主建图 | PASS：连续guided autonomous mapping，6个观测点，无遥控或先验地图 |
| 4. LiDAR与地图增长 | PASS：实际RViz及录像，已知面积2.85→328.84m² |
| 5. 地图主要结构 | PASS：西区、北廊、遮挡隔墙和东区；完整东区95.57%已知，残余遮挡未知区明确保留 |
| 6. 非直线路线 | PASS：实际NavFn A→B概览20.24m，明显绕隔墙；机器人从当前观测位置接续导航，不返回A |
| 7. 动态干扰及恢复 | PASS：真实停等16.10s，撤离后继续7.51m；活跃期扫描命中175帧、costmap标记32帧 |
| 8. 到达B | PASS：位置误差0.09127m，朝向误差0.01396rad，终端重试0次 |
| 9. 四项安全结果 | PASS：静态碰撞0、动态接触0、非足接地0、跌倒0 |
| 10. 停车与清理 | PASS：ACK、物理静止、SLAM生命周期有序关闭、全部组件退出0、残留0、端口42127释放；未使用DDS例外 |
| 11. 现场显示 | PASS：A/B、地图增长、蓝线执行/青线概览/紫线轨迹、等待状态和实际障碍可辨；迟到概览过滤已验证 |
| 12. 真正可播放录像 | PASS：161.50s，646帧全解码，16个关键画面检查 |
| 13. 文档和限制 | PASS：操作、讲解、架构、录像流程、版本身份与限制齐全 |
| 14. Git与证据 | PASS：实现/证据先于完成记录，原标签不变，新demo-v0.2.1标记完整交付；无push |

最终运行：`20261016T090707Z_advanced_d2063f25-32cf-473b-bd06-ff6f2b578c7e`，实测源码`a19286279bfe0122d2a2ffb918b2ac6ae5d89579`，最终实现`405ba357689f566a485d945c797e192b4e981162`，证据`f62f51324500d5d7dc587fe64cd83c21e0043529`。

路线效率：该图形运行采样轨迹29.90m，最终接近15.28sim s；末段路程/端点直线距离比1.144，航向跨度1.201rad，未出现完整末端绕圈。此前同源无界面对照/动态测试分别1.073/1.144；旧发布总行程76.71m，优化后约30m。任务流程、B位置和障碍配置已变化，这不是同条件规划算法基准。步态时序仍可能产生小幅转向修正。

蓝线显示数据：完整运行中11条显示路径与实际/plan序列化字节相同，1条概览消息及其几何未出现在蓝色执行话题。显示状态/轨迹更新5Hz，规划器仍按新目标、路径失效和原5秒周期重规划。阻塞响应为 **BLOCKED_PATH_WAIT_AND_CONTINUE**；本轮Collision Monitor保护停车时间0.00s，不能把规划等待称为紧急保护停车。

验证范围：原4包system colcon构建和194PASS/1历史v0.1源码形状断言排除的完整CPU测试保留；本轮显示/GPU准入改动后18项相关检查PASS，包括异步迟到消息。此前同一导航核心的动态/停放对照与行进中断负测通过；中断任务保持FAILED语义，停车/清理PASS。AST及文件审计确认后续未改导航、控制器、运动和机器人安全行为。v0.1本轮为源码/标签/隔离审计，没有冒称新运行；此前完整U01成功证据保留。W13仍FAILED36/40。

GPU：最小图形检查与完整无录像排练通过；最终录制峰值61°C、1157MiB/6141MiB，无新宿主/图形监控错误。原停机锁逐字节归档，旧1177件证据重新核验。基于53–57°C、704–812MiB、平均28.44%/峰值49%的桌面负载，**仅高级启动准入**从连续5秒≤40%调整为≤60%；默认准入40%不变。运行中75°C、70%显存、增长768MiB、95%持续5秒和错误监控全部原值；未改驱动、显示、环境或录像宿主。

已知限制：预定义观测路线而非frontier exploration；理想真值里程计和真值姿态补偿；动态导航时暂停SLAM扫描匹配/修正，保留最后map→odom并靠理想里程计，LiDAR障碍层仍实时更新，到B后恢复建图；地图存在停放障碍阴影。固定平地/场景/seed、模拟LiDAR与mocap障碍，非任意未知建筑和真机认证；采样间距不是连续时间安全下界；WSLg/NVIDIA OpenGL底层仍为D3D12，监控不能保证故障隔离；DDS历史风险未宣称根治，录像4fps且两窗口顺序采样。

[最终运行](../evidence/advanced-optimization-visual/final-run.json) · [地图覆盖](../evidence/advanced-optimization-visual/final-map-coverage.json) · [完整消息审计](../evidence/advanced-optimization-visual/live-path-topics.json) · [所有本轮尝试](../evidence/advanced-optimization-visual/attempts.json) · [早期优化证据](../evidence/advanced-optimization/README.md)

消息审计说明：首次“所有序列化字节必须相同”断言失败，原报告与退出码保留。进一步核对发现全部23条显示路径的坐标系、逐点坐标/朝向与实际规划一致，11条完整字节也一致。CPU实验复现同一ROS消息在CDR字节40–43存在不同填充值，而反序列化后所有字段相等。因此显示验收采用23条几何匹配、概览隔离及实际画面，未虚称其余12条全部时间戳字段被单独采集核验。详见[对应分析](../evidence/advanced-optimization-visual/live-path-correspondence.json)。

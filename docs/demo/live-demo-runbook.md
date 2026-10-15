# 5–8分钟现场演示

唯一推荐入口：`./scripts/run_demo.sh`。完整操作和应急说明见[quick-start](quick-start.md)。

1. **0:00–0:45，提出任务。** “This is a Booster T1 humanoid walking in MuJoCo. It uses simulated LiDAR and ROS 2 navigation to reach B from A while responding to a moving obstacle.” 指出A/B与橙色障碍，说明没有遥控或基座瞬移。
2. **0:45–2:30，运行一条命令。** 就绪后自动发送目标；MuJoCo看步态与障碍，RViz看地图、scan、局部代价和路径。需要讲解时Alt+Tab切换已有窗口即可。
3. **障碍接近时。** “The obstacle is detected by ray-cast LiDAR. This scenario demonstrates a protective stop, then autonomous resumption when the route clears.” U01是安全等待再继续，不要把它描述成持续阻塞绕行；B01绕行有独立证据。
4. **到达时。** 绿色B点、机器人站稳；最终运行误差0.124m/2.22°。默认8秒后自动清理，等终端结果。提前选择`--hold`可保留最终画面。终端最终判断比截图更完整。
5. **2:30–5:30，解释流程和取舍。** 使用[提纲](presentation-outline.md)讲LiDAR→定位/代价地图→规划→速度安全链→策略/动力学；解释步态死区、姿态补偿、唯一TF/time/command owner，以及图形退出生命周期。
6. **5:30–7:00，证据与限制。** 给出最终运行/录像SHA；披露理想odom、无动态预测、平地原型与W13 36/40 FAILED。关闭DDS异常仅在严格14条件下接受警告，不说“所有异常都可以忽略”。

若演示异常，Ctrl+C等待安全清理；出现图形错误不要重试或改驱动，播放[已验证录像](../evidence/W15/raw/recovery/final-recording-02/demo.mp4)。说明这是发布实现的一次真实记录，不是当前现场成功。录像不含声音，演示者自行讲解。

本版本已完成受控恢复；旧W15阻塞和原始失败仍在[历史审查](../results/w15-original-blocked-review.md)。不创建W16。

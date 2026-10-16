Ubuntu / WSL Bash 终端：

```bash
/home/haoyang/humanoid_nav_ws_advanced/scripts/run_advanced_demo.sh
```

如果提示符以 `PS` 开头，你仍在 Windows PowerShell；进入 `\\wsl$` 文件夹不会切换到 Bash。请使用：

```powershell
wsl.exe -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws_advanced --exec bash ./scripts/run_advanced_demo.sh --hold
```

`--hold` 只保留到达后的画面，启动仍会输出进度。按一次 Ctrl+C 后等待安全停车和清理。

演示材料已集中到 [presentation/](../../presentation/README.md)：[英文命令](../../presentation/COMMANDS.md)、[中英对照讲稿](../../presentation/TALK_TRACK.md)、[B点接口与范围](../../presentation/GOAL_CONFIGURATION.md)。每次启动会先读取 `presentation/goal.yaml`；可用 `./scripts/run_advanced_demo.sh --check-goal` 检查，修改坐标无需构建。当前接口已完成默认B与边界B=(11,0.5)的完整验证，见[接口验证](../../presentation/VALIDATION.md)。下述旧发布录像保持默认B=(12,0)。

**ADVANCED_DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**。当前优化发布为 `demo-v0.2.1`：已完成完整图形演示、真实动态停等/恢复、录像完整解码和关键画面审查。旧 `demo-v0.2.0` 与稳定兜底 `demo-v0.1.0` 保持原标签和历史。

机器人从 A=(0,0) 出发，自主执行 **guided autonomous mapping**：沿西区、7.5米宽北廊和遮挡后的东区连续通过观测点。第5点起检查实际地图主通道覆盖率≥95%，且真实 NavFn 能在已观测地图上规划 A→B；必要时补第6个观测点。不返回 A，也不逐点停稳校正朝向。地图足够后从当前位置继续走向 B=(12,0)，以高级专用 Regulated Pure Pursuit 跟踪路径。

橙色移动障碍宽3米，以0.6m/s横向进入通路，停留8秒再撤离。最终录像对应运行实际停等 **16.10秒**，撤离后继续 **7.51米**，最终到达并停车。障碍由真实 LiDAR/costmap 感知，没有由障碍真值发送停车指令。

RViz：**蓝色**为最新实际 Nav2 执行段路径，附路径年龄；**青色**为建图后计算的完整 A→B 概览，仅供展示；**紫色**为实际轨迹；**橙色十字**为目标。等待时显示 Waiting / replanning，保留的蓝线不保证此刻仍可通行。状态/轨迹5Hz更新，规划按新目标、路径失效或原5秒周期更新。已核对完整运行中11条执行路径字节一致，概览没有混入蓝线。

按一次 Ctrl+C 请求锁存停车和清理，等待终端 `Final SUCCESS` 及端口释放；TCP TIME_WAIT 最多可能再等65秒。`--hold` 可延长最终画面，正常入口不录制。录像与复核结果见 [当前发布清单](optimization-release-manifest.md)。

必须说明的限制：预定义观测路线，不是 frontier exploration；使用理想真值里程计与真值姿态补偿。动态导航阶段暂停 SLAM 测量及定位修正，沿用最后 map→odom 和理想里程计，LiDAR/costmap避障仍实时工作；到 B 后恢复 SLAM 并保存地图。主要结构已建立，停放障碍后方仍有少量未知区，完整东区覆盖约95.57%。不是任意未知建筑探索或真机可靠性认证。

稳定兜底：`/home/haoyang/humanoid_nav_ws/scripts/run_demo.sh`。

[验收结果](optimization-review.md) · [讲解提纲](presentation-outline.md) · [架构](architecture.md) · [独立录像](recording.md)

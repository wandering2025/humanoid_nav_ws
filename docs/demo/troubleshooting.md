# 现场排障

当前 **W14_DEMO_READY_WITH_DDS_TEARDOWN_WARNING**，GUI已解除本次DDS-only封锁。
第二轮原始FAILED/−11保留；不是“DDS已修好”。[严格发布例外](dds-teardown-exception.md)要求全部14项门槛，任一缺项仍失败。
运行期DDS死亡/ROS丢失、未到达、未停车、机器人/图形组件异常、需SIGTERM/SIGKILL、残留、宿主故障或日志/退出码缺失均不能放行。
禁止驱动/BIOS/注册表/显卡模式/系统显示改动；新图形错误停止并保存证据，不自动重试同一路径。本次W14不再运行GPU/GUI。

| 问题 | 拟检查 |
|---|---|
| 端口占用 | 先区分LISTEN与TIME_WAIT。只有TIME_WAIT时等待内核自然到期，不杀进程；清理现在有界等待65s。`ss -ltnp 'sport = :42127'`识别owner；只停止确认属于自己的原演示。 |
| DDS退出−11 | 若导航已成功、安全停车、其余组件全0、DDS仅预期SIGINT后退出、无运行期通信问题、无强杀/残留/端口/宿主故障且日志与精确码保存，显示DDS_TEARDOWN_WARNING并返回成功；JSON仍为−11。任一条件缺失判FAILED。外部libfastrtps TCP线程崩溃未修复，不屏蔽stderr。 |
| 遗留进程 | 比对PID/ppid/command与owned-processes.json；优先原终端Ctrl+C，不pkill ROS/Python。 |
| RViz无图/退出−6 | 查rviz.log；本次corrupted double-linked list为SIGABRT，不改写0。不要切回原MapDisplay shader或直接重跑。 |
| MuJoCo未出现 | launch.log中模型/策略/GL错误；本事故D3D12 Removing Device必须停，不自动切卡重试。 |
| ROS未就绪 | starter/initial_pose/lifecycle日志；有界拒绝目标，保留失败而非手工强制arm。 |
| 动态体未启动 | U01须实际走过x=2.9m；核对phase/scan hits，不能改触发参数假装成功。 |
| 目标失败 | 保留goal.log/result，禁止换seed、放宽容差或任务时间。导航失败W14_BLOCKED。 |
| 安全停止 | CM可等待障碍退离；gate STOPPED为锁存，不补速度或自动复位。 |
| 完整重启 | 先确认安全退出、所有登记PID消失及端口释放；只有无新图形故障且上次完整清理、端口已释放才可完整重启；合规DDS关闭警告不等于进程残留。任何新的halt先审查，不直接删除或自动重试。 |
| 备用录像 | 仅播放manifest哈希匹配、可播放已核验的成功原片并口播“预录”；当前FINAL_VIDEO_PENDING_W15，没有可播放备份。 |

日志为run_data/demo/<run_id>/，仿真原始记录为run_data/<run_id>/。事故强制重启后的无残留不能代替脚本清理验收。事故MuJoCo/RViz最终退出整数未恢复，不能写0。
后续CPU辅助run的DDS/launch/goal及17组件均exit0，安全stop/站稳通过；原立即端口bind失败和动态0.42s失败仍保留，不能改成PASS。

人工录像状态 **FINAL_VIDEO_PENDING_W15**，由W15在其授权范围内执行，当前文件/大小/SHA256 N/A，播放NOT_RUN；W14不再开窗口或安装工具：

1. 确认同一发布分支、源码clean，记录 `git rev-parse HEAD`；由W15核验当前发布例外、无新图形halt及安全边界后再安排录制。
2. 使用用户已安装、支持桌面矩形区域录制的工具。若没有此类工具，保持FINAL_VIDEO_PENDING_W15，另行安排工具，不用单窗口片段冒充双视图。
3. 在工具中选择包含MuJoCo、RViz及终端的同一桌面区域，保留原显示分辨率；逐窗安排可见位置，不修改系统显示设置。选30fps（若工具支持），不做倍速。
4. 开始录制，保留约3秒；在全新Ubuntu终端输入唯一命令 `./scripts/run_demo.sh`，录下run ID、启动等待及真实就绪过程。
5. 持续录下实体接近、LiDAR/costmap变化、停车/等待、恢复、规划与到达。保留全部关键等待与末端修正，不剪掉失败。
6. 到达后讲解双视图，按Ctrl+C；继续录下独立结果、真实退出码、端口释放。最终只有SUCCESS或满足严格14项门槛的SUCCESS_WITH_WARNING才可作为演示原片；DDS警告及精确码必须录入，其他失败原片保留，不列为成功备份。
7. 确认无残留并再保留3秒后停止录制。原片保存到ignored `run_data/demo-video/<run_id>.mp4`，不覆盖已有文件。
8. 在Ubuntu中执行 `stat -c '%s' run_data/demo-video/<run_id>.mp4` 与 `sha256sum run_data/demo-video/<run_id>.mp4`；记录run ID、代码SHA、U01/4000、文件路径、字节数及SHA256到release manifest。
9. 用已有播放器从头至尾播放原片，确认两窗口与终端可读、等待完整、无黑帧或截断、速度正常；记录播放器/核验时间/确认人，验证后才标PLAYBACK_VERIFIED。
10. 现场切换备用时先停止并清理live系统，再播放哈希匹配的成功原片，明确口播“预录”。当前没有可用成功录像，路径/大小/hash均N/A。

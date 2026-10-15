# 现场排障（当前W14_BLOCKED）

当前GUI因第二轮DDS退出−11及授权次数耗尽封锁，不清除halt自动重跑。`--preflight`为隔离CPU资源检查；不是GPU恢复证明。
本轮两次同源码都已到达，RViz都正常退出0；第二轮DDS关闭段错误导致整体FAILED。版本与run ID见[清单](release-manifest.md)。
禁止驱动/BIOS/注册表/显卡模式/系统显示改动。图形错误停止扩载、终止最近任务、保存结果，不自动重试同一路径。

| 问题 | 拟检查 |
|---|---|
| 端口占用 | 先区分LISTEN与TIME_WAIT。只有TIME_WAIT时等待内核自然到期，不杀进程；清理现在有界等待65s。`ss -ltnp 'sport = :42127'`识别owner；只停止确认属于自己的原演示。 |
| DDS退出−11 | 内核已记录fast-discovery-server的dds.tcp.42127线程在libfastrtps内段错误；不改写0，不清锁追加第三轮。保存dds.log/内核/result并离线调查关闭顺序。 |
| 遗留进程 | 比对PID/ppid/command与owned-processes.json；优先原终端Ctrl+C，不pkill ROS/Python。 |
| RViz无图/退出−6 | 查rviz.log；本次corrupted double-linked list为SIGABRT，不改写0。不要切回原MapDisplay shader或直接重跑。 |
| MuJoCo未出现 | launch.log中模型/策略/GL错误；本事故D3D12 Removing Device必须停，不自动切卡重试。 |
| ROS未就绪 | starter/initial_pose/lifecycle日志；有界拒绝目标，保留失败而非手工强制arm。 |
| 动态体未启动 | U01须实际走过x=2.9m；核对phase/scan hits，不能改触发参数假装成功。 |
| 目标失败 | 保留goal.log/result，禁止换seed、放宽容差或任务时间。导航失败W14_BLOCKED。 |
| 安全停止 | CM可等待障碍退离；gate STOPPED为锁存，不补速度或自动复位。 |
| 完整重启 | 先确认安全退出、所有登记PID消失及端口释放；只有无新图形故障且上一次正常清理才可完整重启；halt文件存在时先审查原因，不直接删除。 |
| 备用录像 | 仅播放manifest哈希匹配、可播放已核验的成功原片并口播“预录”；当前MISSING。 |

日志为run_data/demo/<run_id>/，仿真原始记录为run_data/<run_id>/。事故强制重启后的无残留不能代替脚本清理验收。事故MuJoCo/RViz最终退出整数未恢复，不能写0。
后续CPU辅助run的DDS/launch/goal及17组件均exit0，安全stop/站稳通过；原立即端口bind失败和动态0.42s失败仍保留，不能改成PASS。

人工录像是解除W14阻塞后、W15冻结前必须完成的动作；当前未录制、未验证播放，不安装工具：

1. 确认同一发布分支、源码clean，记录 `git rev-parse HEAD`；当前GUI封锁时不得执行以下录制。
2. 使用用户已安装、支持桌面矩形区域录制的工具。若没有此类工具，保持录像MISSING，另行安排工具，不用单窗口片段冒充双视图。
3. 在工具中选择包含MuJoCo、RViz及终端的同一桌面区域，保留原显示分辨率；逐窗安排可见位置，不修改系统显示设置。选30fps（若工具支持），不做倍速。
4. 开始录制，保留约3秒；在全新Ubuntu终端输入唯一命令 `./scripts/run_demo.sh`，录下run ID、启动等待及真实就绪过程。
5. 持续录下实体接近、LiDAR/costmap变化、停车/等待、恢复、规划与到达。保留全部关键等待与末端修正，不剪掉失败。
6. 到达后讲解双视图，按Ctrl+C；继续录下独立结果、真实退出码、端口释放。最终不是SUCCESS则保留失败原片，不把它列为成功备用片。
7. 确认无残留并再保留3秒后停止录制。原片保存到ignored `run_data/demo-video/<run_id>.mp4`，不覆盖已有文件。
8. 在Ubuntu中执行 `stat -c '%s' run_data/demo-video/<run_id>.mp4` 与 `sha256sum run_data/demo-video/<run_id>.mp4`；记录run ID、代码SHA、U01/4000、文件路径、字节数及SHA256到release manifest。
9. 用已有播放器从头至尾播放原片，确认两窗口与终端可读、等待完整、无黑帧或截断、速度正常；记录播放器/核验时间/确认人，验证后才标PLAYBACK_VERIFIED。
10. 现场切换备用时先停止并清理live系统，再播放哈希匹配的成功原片，明确口播“预录”。当前没有可用成功录像，路径/大小/hash均N/A。

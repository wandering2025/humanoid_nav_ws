# 现场排障

目前默认正式排练未结束，发布状态以[清单](release-manifest.md)为准。已验证NVIDIA MuJoCo+CPU RGB网格RViz的站立/停车/退出。
不修改驱动、BIOS、注册表、显卡模式或系统显示。出现图形错误、超时或持续无响应，停止扩载并终止最近任务、保留run；不自动重试相同路径。

| 问题 | 拟检查 |
|---|---|
| 端口占用 | 先区分LISTEN与TIME_WAIT。只有TIME_WAIT时等待内核自然到期，不杀进程；清理现在有界等待65s。`ss -ltnp 'sport = :42127'`识别owner；只停止确认属于自己的原演示。 |
| 遗留进程 | 比对PID/ppid/command与owned-processes.json；优先原终端Ctrl+C，不pkill ROS/Python。 |
| RViz无图 | launch.log、fixed frame map及图形错误；进程存在不证明渲染正常。 |
| MuJoCo未出现 | launch.log中模型/策略/GL错误；本事故D3D12 Removing Device必须停，不自动切卡重试。 |
| ROS未就绪 | starter/initial_pose/lifecycle日志；有界拒绝目标，保留失败而非手工强制arm。 |
| 动态体未启动 | U01须实际走过x=2.9m；核对phase/scan hits，不能改触发参数假装成功。 |
| 目标失败 | 保留goal.log/result，禁止换seed、放宽容差或任务时间。导航失败W14_BLOCKED。 |
| 安全停止 | CM可等待障碍退离；gate STOPPED为锁存，不补速度或自动复位。 |
| 完整重启 | 先确认安全退出、所有登记PID消失及端口释放；只有无新图形故障且上一次正常清理才可完整重启；halt文件存在时先审查原因，不直接删除。 |
| 备用录像 | 仅播放manifest哈希匹配、可播放已核验的成功原片并口播“预录”；当前MISSING。 |

日志为run_data/demo/<run_id>/，仿真原始记录为run_data/<run_id>/。事故强制重启后的无残留不能代替脚本清理验收。事故MuJoCo/RViz最终退出整数未恢复，不能写0。
后续CPU辅助run的DDS/launch/goal及17组件均exit0，安全stop/站稳通过；原立即端口bind失败和动态0.42s失败仍保留，不能改成PASS。

人工录像仍是恢复后、W15冻结前必须完成的动作：在同一发布SHA与固定U01/4000下，
用用户已有桌面录屏工具同时录下MuJoCo、RViz和终端，从启动至等待/恢复/到达/退出，
不剪失败或关键等待；成功原片放ignored run_data/demo-video/<run_id>.mp4。
记录 `stat -c '%s' <file>`、`sha256sum <file>`、run ID和代码SHA；用播放器完整回放，
记录核验时间/播放器/确认人。当前无ffmpeg/ffprobe，未自动录制，未验证可播放。

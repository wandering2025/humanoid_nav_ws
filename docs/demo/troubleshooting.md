# 现场排障（当前禁止启动）

宿主黑屏事故未解除，**不要试图换显卡或软件图形窗口绕过默认GUI封锁**。
续作仅开放Landlock保护的CPU预检和--headless；它们没有显示窗口，不能当作GUI已恢复。
先读[安全审查](graphics-incident-20261015.md)，不改TDR注册表/驱动/依赖来碰运气。
以下仅为恢复审查后的操作草案。

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
| 完整重启 | 先确认安全退出、所有登记PID消失及端口释放；当前事故后不得重启演示。 |
| 备用录像 | 仅播放manifest哈希匹配、可播放已核验的成功原片并口播“预录”；当前MISSING。 |

日志为run_data/demo/<run_id>/，仿真原始记录为run_data/<run_id>/。事故强制重启后的无残留不能代替脚本清理验收。事故MuJoCo/RViz最终退出整数未恢复，不能写0。
后续CPU辅助run的DDS/launch/goal及17组件均exit0，安全stop/站稳通过；原立即端口bind失败和动态0.42s失败仍保留，不能改成PASS。

人工录像仍是恢复后、W15冻结前必须完成的动作：在同一发布SHA与固定U01/4000下，
用用户已有桌面录屏工具同时录下MuJoCo、RViz和终端，从启动至等待/恢复/到达/退出，
不剪失败或关键等待；成功原片放ignored run_data/demo-video/<run_id>.mp4。
记录 `stat -c '%s' <file>`、`sha256sum <file>`、run ID和代码SHA；用播放器完整回放，
记录核验时间/播放器/确认人。当前无ffmpeg/ffprobe，未自动录制，未验证可播放。

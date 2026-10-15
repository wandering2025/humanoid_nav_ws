# W14 DDS_TEARDOWN_EXCEPTION — 用户明确授权的演示发布决策

2026-10-15；起始分支 release/w13-formal-v1-demo，HEAD 61644012094a251b41e66256d4bb4f1e848f67c5，工作区干净。
用户以“求职演示优先”授权收尾，仅改变演示结束后的 DDS 退出分类，不再修复外部 Fast DDS 2.14.7 TCP 线程 SIGSEGV。
本决策作为独立提交，SHA 由 `git log -- docs/demo/dds-teardown-exception.md` 获取。

发布状态：**W14_DEMO_READY_WITH_DDS_TEARDOWN_WARNING**（实现、非图形测试和交接完成后生效）。
首次 W13 **FAILED** 不变，基线接受决策 ebbf062da8db33c691935a64ab327fbfccbd122f 不变；不是安全认证或可靠性声明。

## 必须同时成立的边界

1. 导航结果已经为 SUCCESS（仍须到达、动态判据 PASS、零碰撞和跌倒）。
2. 机器人安全停车 ACK 且物理静止完成。
3. navigation_starter、launch、MuJoCo、RViz、目标工具及全部机器人组件退出 0。
4. DDS 运行期间未提前死亡。
5. 导航期间 ROS 通信未中断，原 clock/图/初始化检查通过。
6. DDS 仅在收到预期的关闭 SIGINT 后退出。
7. DDS 完整日志保存在本次目录并记录哈希。
8. DDS 精确非零退出码保存在结果中，尤其 `dds_returncode=-11`，不伪造子进程退出。
9. 不需要 SIGTERM 或 SIGKILL；任何关闭升级均阻止例外。
10. DDS PID 已消失。
11. DDS 端口已释放。
12. 没有本项目进程残留。
13. 没有宿主图形故障、黑屏；保留 Windows/WSLg 检查证据。
14. 终端明确显示 **DDS_TEARDOWN_WARNING** 和精确退出码。

任一条件不成立或证据缺失，仍为 FAILED；不允许把运行期 DDS 死亡、通信丢失、导航或停车失败、其他组件异常、强杀、残留、宿主故障归入例外。
不屏蔽 stderr、不删除失败、不伪造 0。原始结果与新发布分类分开保存。

## 已有两轮与验证范围

两轮同一干净 a963518410ad0faefad0c3b2230d367780c91212，U01/seed4000，命令均 `./scripts/run_demo.sh`。
完整01原 SUCCESS；完整02原 FAILED 永久保留。新汇总仅将02解释为 NAVIGATION_SUCCESS / SAFETY_PASS / DDS_TEARDOWN_WARNING。
完整02内核定位到 libfastrtps.so.2.14.7 的 dds.tcp.42127 线程，具体库内缺陷没有栈，不能声称已经修复。
首次黑屏、CPU 动态判据失败、RViz −6 和本次 DDS −11 的全部证据及原索引保持不变。
本次只运行受影响的非图形单元/分类/脚本静态检查，不启动 GPU 查询、窗口、物理仿真或新增完整排练。
解除本次 DDS-only GUI 封锁时原 halt 原字节归档；任意新的图形故障锁仍不可自动清除。
不修改导航、控制、安全阈值、世界、轨迹、起终点、seed、驱动、BIOS、注册表、显卡模式或系统显示。

录像状态 **FINAL_VIDEO_PENDING_W15**：当前文件路径、大小和 SHA256 均 N/A，播放未验证；W15负责录制并核验完整双视图原片。
W14不启动W15。W15仍需视觉优化、独立复验及版本冻结，并阅读最终交接。
监控不是宿主故障隔离保证；此前 dxg adapter-query ioctl 错误仍披露为未证明无害。

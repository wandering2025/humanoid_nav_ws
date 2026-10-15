# 演示故障处理

正常入口 `./scripts/run_demo.sh`；不要在失败后自动改seed、阈值或渲染后端。

| 现象 | 操作 |
|---|---|
| 缺资源/解释器/overlay | 先读输出与environment.log；使用scripts/env.sh中的冻结路径。不要重装或升级环境掩盖错误。CPU预检不打开GPU。 |
| 端口未释放 | 等原运行的有限TCP冷却；若仍占用，检查拥有该端口的终端，不杀全部ROS/Python。 |
| 到达后窗口退出但终端仍等待 | 正常：finalizer等ROS退出后的端口释放，最多65s。 |
| 需要马上停车 | 在本次启动终端Ctrl+C一次，等待锁存停止/清理；不要先关闭终端。 |
| GPU halt、窗口持续无响应、驱动/图形错误 | 停止新增负载，保留日志和锁，不自动重试、不改驱动或系统显示；使用已验证录像。 |
| DDS_TEARDOWN_WARNING | 必须已有导航/动态/安全成功，全部其他组件退出0，且14项例外全部满足。真实DDS码保留；不把其他失败归为警告。 |
| RViz或MuJoCo遮住另一窗口 | 用Alt+Tab切换；导航不需鼠标发送目标。 |
| 独立录像未生成 | 演示与录制互不依赖；检查capture.json/frames.jsonl，不把零帧尝试或诊断片当成功录像。发布已有可播放原片。 |

日志：`run_data/demo/<run_id>/`；物理记录：`run_data/<run_id>/`。[最终录像与SHA256](release-manifest.md)。所有历史失败仍可追溯；W15开发在发布交接后结束，无W16。

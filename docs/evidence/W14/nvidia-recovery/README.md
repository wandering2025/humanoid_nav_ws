# W14 用户授权NVIDIA恢复：按停止条件收尾，仍BLOCKED

用户明确允许RTX4050分阶段工作、一次一个GPU密集型进程，图形错误立即停止；不准改驱动/BIOS/注册表/显示设置。
另授权保留CPU失败，重新预登记两次同seed/导航参数的默认GUI排练。预登记docs/demo/nvidia-recovery-plan.md。

| 阶段 | 实测 | 结果 |
|---|---|---|
| NVML初始 | 47°C，637/6141MiB，21% | 只读采集正常 |
| 首次GL启动门 | 46°C、589MiB、41% | 超过40%启动线，未创建上下文 |
| 最小GL | 实际NVIDIA4050，exit0，最高47°C/591MiB/27% | 原解析器把标题误作renderer导致FAIL；原结果保留，离线修正仅此检查为PASS，不重跑GL或改导航结果 |
| 320×240/10fps/15s | exit0，最高49°C/600MiB/44% | PASS |
| 静态官方T1/800×600/15fps/30s | exit0，最高53°C/708MiB/58% | PASS；无物理/策略/导航 |
| 站立检查01 | 88ebd97d5b282b891d9d4808485d3a31cdc928f3；最高61°C/764MiB/49% | 未发目标；FAIL，详下 |
| 站立检查02 | 7074997f741e39081920f69dcef340ef87123bfb；最高60°C/787MiB/49% | 未发目标；FAIL，详下 |

站立run IDs见summary.json。MuJoCo使用NVIDIA，随后RViz受Landlock GPU设备限制、实际llvmpipe；未并发两个GPU渲染器。
两次真实画面均已截图并查看，第二次安全圆、绿色目标和局部footprint可见，但MarkerArray显示仍有error图标，不能当作最终视觉验收通过。
两次安全stop ACK/站稳通过、接触三类0、跌倒False、策略命令全0；DDS/launch/RViz和17个launch子进程均exit0。
不过两次原端口检查均在ROS runner进程尚存时进行，观测FIN_WAIT2，保留失败；事后进程/端口审计无残留且可bind。
最后修复将核验移到独立标准库finalizer，原始result永不覆盖，意外退出/动态失败不能被finalizer改成成功；仅单元覆盖，未再GPU复验。

更重要的是，RViz日志两次均有GLSL sampler link error。之前监控只覆盖设备失效/超时等，漏检此类shader错误，
是在第二次站立检查结束后的日志审查中发现；不能声称本次已实时捕捉。发现后立即保持GPU停止，写入持续halt文件并锁定profile。
补充了GLSL/GL_INVALID/EGL_BAD错误检测及回归检查，但没有再开窗口验证。无新导航物理排练、无换seed或阈值变化。
[RViz上游相同错误记录](https://github.com/ros2/rviz/issues/463)不能证明本机可以忽略；按用户更严格的停止条件处理。

测试时段Windows事件只读查询：0条匹配Display/LiveKernelEvent/Kernel-Power41，查询errors为空。
这不是绝对硬件安全证明；只能说明这次查询没有发现新的该类宿主事件。没有更改驱动/BIOS/注册表或系统显示。

新授权GUI正式01/02均NOT_RUN，视频MISSING，W15未启动。GPU工作保持停止，下一步仅离线或受限CPU调查RViz shader与显示状态，
确认修复并审查恢复条件前不得清除halt或直接重跑。W13 FAILED、CPU辅助FAIL、旧事故和实验分支完整保留。

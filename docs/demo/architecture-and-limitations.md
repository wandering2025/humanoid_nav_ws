> **W15 当前状态：FINAL_RELEASE_BLOCKED（2026-10-15）。** 新最小 MuJoCo 检查退出 −11，内核记录 NVIDIA `libnvwgf2umx.so` 故障；GUI 已在 GPU 访问前封锁。独立 B01 也发生 120s 超时。没有成功的最终录像，不可将本工程声明为现场发布完成。详见 [最终审查](../results/final-review.md)。以下 W14 内容保留为历史或讲解草案，不代表当前启动许可。

# 架构与已知限制

当前 **W14_DEMO_READY_WITH_DDS_TEARDOWN_WARNING**；严格[DDS关闭例外](dds-teardown-exception.md)不代表修复DDS或认证安全。历史[宿主图形事故](graphics-incident-20261015.md)仍保留。

MuJoCo唯一物理owner以2ms推进接触动力学，20ms执行官方t1_walk策略；不编辑基座伪造行走。
真实9×360射线生成scan；AMCL在已有SLAM图上定位且独占map→odom。NavFn全局规划，
RotationShim+MPPI跟踪，经适配器→gate→smoother→Collision Monitor→只减不增bridge→策略。
动态mocap实体进入射线与碰撞；规划器只读scan/costmap，不读实体真值或未来轨迹。
评测器独立读取truth；离线独立模型重放不驱动现场机器人。详见[导航契约](../architecture/navigation.md)。

首次W13仍FAILED：static10/10、crossing8/10、blocking8/10、sudden10/10，总36/40，
环境碰撞/跌倒0；C01启动定位、C09 action/服务确认超时、B04/B06末端过冲/收敛失败。
headless RTF 0.9999998608211204严格低于1，工程差异小仍记FAIL。
原[评测](../results/evaluation.md)/[性能](../results/performance.md)不改，
仅[用户决策](baseline-decision.md)允许W14使用该失败基线。

限制：理想truth odom、真值姿态补偿、默认无噪声、有限垂直束会漏检束间/高度外物体；
平地宽通道（规划半径2.05m）、不可推开的mocap、反应式避障无动态预测；低速死区/停车偏航回弹；
ROS/MPPI调度非确定性；同机小样本不能外推真机可靠率。W10停止/减速圆1.65/1.95m保持。
10Hz采样三维全身净距不是连续下界，500Hz接触另计。无硬件安全认证。

当前图形限制：历史双GUI触发/暴露宿主驱动TDR与黑屏；现在MuJoCo和RViz明确绑定NVIDIA RTX4050，WSLg仍是Mesa/D3D12。
RViz RGB点格薄宿主按Qt线程顺序销毁显示和ROS context；最小和两次完整退出均0，但原−6的精确堆破坏对象未证明。
第二轮DDS关闭后外部libfastrtps.so.2.14.7 TCP线程SIGSEGV−11未修复，严格门槛满足时仅列关闭警告，原FAILED不改。
无新增Windows显示事件/LiveKernelEvent141或WSLg RemovingDevice/Pagefault；kernel仍有dxg adapter-query ioctl−22/−2，未证明无害。
正常进程回收和监控不构成宿主故障隔离保证，不能承诺绝对安全；新图形故障仍锁存并停止，不自动重试。
RViz只读原地图/costmap，转换单元格中心与颜色供显示；不改变planner输入，原MapDisplay shader禁用。
录像FINAL_VIDEO_PENDING_W15，无已验证备用片。本次不执行W15。

不打包外部权重/mesh。冻结Deploy 7bb1462，LICENSE标Apache-2.0并提及IsaacLab BSD-3-Clause；
Assets 12a9751，BSD-3-Clause。本地文件清单未发现更局部许可证；不是对所有第三方/权重分发权的法律结论。
按[固定来源](../../dependencies.repos)获取，分享时保留版权/许可说明；本次不重新分发这些资产。
[资产清单](../baseline/assets-manifest.md)。


CPU续作：在任何runtime导入前用Landlock禁止GPU设备read/write/ioctl，子进程exec继承；
不探测CUDA、不创建窗口。原官方策略本就在CPU，Nav2/物理/安全参数不变。
唯一辅助run已到达、零接触/跌倒、stop和正常退出通过；保护区连续静止0.42s不足冻结0.5s，
整轮仍FAIL。CPU辅助原FAIL不变。GUI最新到达/动态通过但RViz异常退出；两次默认验收及录像仍未完成。

本轮最新阻塞：CPU RViz退出原生堆损坏（−6），具体库/线程根因未定位。当前GUI再次锁定，停止扩载且未自动重试。
本轮Windows查询无匹配新驱动/异常断电事件，不足以排除底层驱动因素。30项非图形回归检查不证明再次渲染安全。

最新严格授权结果：仓库本地RViz薄宿主在ROS context有效时逐个销毁显示，再shutdown ROS和frame，明确NVIDIA，15Hz显示。
唯一最小+两轮RViz均0；两轮导航/dynamic均PASS，但第二轮DDS TCP线程关闭SIGSEGV−11。当前blocker为DDS生命周期；
精确RViz破坏堆对象仍未知，不把三次退出0当作根因单因素证明。内核dxg ioctl−22/−2也如实披露，未证明无害。

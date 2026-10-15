# 架构与已知限制

当前 **W14_BLOCKED**，见[宿主图形事故](graphics-incident-20261015.md)。

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

当前图形限制：历史双GUI触发/暴露宿主双显卡TDR。已改为明确NVIDIA MuJoCo+CPU llvmpipe RGB点格RViz；
替代组合站立/停车/退出/端口已通过，完整默认正式排练尚待完成。正常进程回收和监控不能阻止宿主驱动挂起。
RViz只读原地图/costmap，转换单元格中心与颜色供显示；不改变planner输入。原MapDisplay shader禁用。

不打包外部权重/mesh。冻结Deploy 7bb1462，LICENSE标Apache-2.0并提及IsaacLab BSD-3-Clause；
Assets 12a9751，BSD-3-Clause。本地文件清单未发现更局部许可证；不是对所有第三方/权重分发权的法律结论。
按[固定来源](../../dependencies.repos)获取，分享时保留版权/许可说明；本次不重新分发这些资产。
[资产清单](../baseline/assets-manifest.md)。


CPU续作：在任何runtime导入前用Landlock禁止GPU设备read/write/ioctl，子进程exec继承；
不探测CUDA、不创建窗口。原官方策略本就在CPU，Nav2/物理/安全参数不变。
唯一辅助run已到达、零接触/跌倒、stop和正常退出通过；保护区连续静止0.42s不足冻结0.5s，
整轮仍FAIL。默认GUI、两次默认排练及录像仍未通过；不能用headless成功到达掩盖动态判据失败。

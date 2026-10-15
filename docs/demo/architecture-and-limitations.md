# 架构与已知限制

当前 **DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**，依据[最终审查](../results/final-review.md)。固定U01演示已验证，W13正式FAILED不变；不是生产级可靠性或硬件安全认证。

MuJoCo唯一物理owner以2ms推进接触动力学，20ms执行官方t1_walk策略和关节PD；不编辑基座来行走。9×360几何射线生成scan，真实动态实体进入射线与碰撞。SLAM Toolbox生成并保存地图；当前AMCL独占map→odom，理想odom提供odom→base_footprint。NavFn与RotationShim/MPPI经适配器→gate→smoother→Collision Monitor→只减不增bridge→策略。规划器不读取障碍真值或未来轨迹；评测器独立用真值判定。

A/B标记仅在独立viewer副本的user_scn与只读RViz MarkerArray中存在。它们不进入MjModel、碰撞、LiDAR、地图、costmap或导航结果判定。显示模型的简单颜色/已有headlight不改变物理模型。恢复版保留W14跟随与close+join，放弃原W15网格/全路线相机/ctypes窗口管理/自动双窗录像；录像是独立ImageMagick抓图，关闭仿真后CPU编码。

U01的避障方式是安全等待后继续，保护区实际静止7.08s；不是持续阻塞绕行。独立B01恢复检查有真实绕行，先前一次B01超时也完整保留。冻结阈值未降低，W10停止/减速圆仍1.65/1.95m。

已知限制：理想truth odom、真值姿态补偿、有限垂直束/默认零噪声、宽平走廊、不可推开的mocap障碍、反应式避障无预测、策略低速死区与停车偏航回弹、ROS/MPPI调度非确定性。10Hz三维全身净距不是连续下界，接触另按500Hz物理步检查。

W13仍FAILED：static10/10、crossing8/10、blocking8/10、sudden10/10，共36/40，碰撞0/跌倒0。C01启动定位、C09 action/服务确认超时、B04/B06末端收敛失败保留；headless中位RTF0.9999998608211204严格低于1的原判定也不改。[原评测](../results/evaluation.md)。

图形仍使用本机WSLg Mesa/D3D12的NVIDIA RTX4050适配器，没有转用Intel、安装驱动或改变系统显示。恢复后未出现新的gdrv/libnv崩溃、WSLg设备丢失或Windows显示事件，但已有dxg adapter/feature查询−22/−2日志持续存在，不能宣称普遍无害。旧W15SIGSEGV、历史黑屏和RViz异常退出证据均保留。新故障会锁存，禁止自动重试；监控不是宿主故障隔离保证。

DDS外部Fast DDS2.14.7关闭缺陷未修复。恢复录制尝试01在期望SIGINT后退出−11，全部14门槛满足，分类为SUCCESS_WITH_WARNING；最终录像运行所有组件退出0。[例外全文](dds-teardown-exception.md)不变，运行期通信、导航/安全、其他组件或宿主图形错误不能套用。

录像77.125s，1024×736/8fps，真实抓取418帧，以重复最近帧保留墙钟时间，没有加速；末尾8s为明确标注的运行后结果卡，没有音频。无额外媒体播放器GPU负载；用软件完整解码和关键帧视觉检查验证。环境为现有本机安装，原片/原始trace保持ignored并由哈希索引追溯，不声称克隆仓库即可在任意GPU/空白系统运行。

不重新分发外部权重和mesh；固定来源见[dependencies](../../dependencies.repos)与[资产清单](../baseline/assets-manifest.md)。冻结外部仓库未修改。旧W15[阻塞审查](../results/w15-original-blocked-review.md)、[阻塞发布清单](w15-original-blocked-manifest.md)完整保留。

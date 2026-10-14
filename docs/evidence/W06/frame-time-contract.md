# W06 C1 frame/time implementation plan

沿用 W02 interfaces §4/5/8/9，无语义修订。

- 固定名称集中在状态桥模块：odom / base_footprint / base_link；MJCF body=trunk（已由 model_contract 核验 world_joint 所属及地址）。base_link 为 trunk XBODY/free joint 原点，非质心。RSP 包装冻结上游 T1_23dof.urdf，加 base_link→trunk identity；不改上游，不发布 map 或未来 LiDAR frame。
- base_footprint 为 trunk 原点在 z=0 平面的竖直投影，姿态为 yaw。投影导数是世界水平线速度；它不是固定杆臂，所以不加 ω×offset。odom twist 用 yaw 旋转到 child 坐标；wz 为世界角速度竖直分量，保持 W05 v_meas 定义。
- 现场 MuJoCo 3.15.0 探针：倾斜 + 90° 附近 yaw + 混合线/角速度，mj_objectVelocity(mjOBJ_XBODY) 与上述 qvel 语义一致；COM 速度确实不同。见 velocity-probe.json / probe_velocity.py。
- 每步只在仿真所有者线程复制 qpos/qvel/实际 actuator effort；同一不可变快照产生 clock、odom、TF、joints、6D truth 的相同整数纳秒 stamp。ROS 回调仅写既有 input/request buffer。
- clock 沿用 W04 唯一发布者；不创建模拟 ROS 定时器，启动 clock=0，仿真时间分频使用步计数。odom/TF/truth 50 Hz，joints 默认 50 Hz，可配置正整数分频（契约允许，选 50 而非近似 30）。
- t1_sim use_sim_time=false，RSP/RViz/测试消费者 true。暂停不发布动态状态；状态门立即清命令，墙钟心跳继续。resume 必须新命令。sim_only reset 保持 clock 单调、清发布分频；episode 状态通知测试消费者清 TF 缓存。跨 run 需要整套进程重新启动。
- odom 是理想真值投影，1e-6 对角协方差是名义值，非估计精度。6D truth 只供评测，不进 TF。首版不发布可选 IMU；未实现 LiDAR/SLAM/导航。

探针首次 exit 1：把上游实例方法误作静态方法调用；读取源码后改为 cfg 路径占位符展开，第二次 exit 0，5/5 PASS；未改环境或上游。

# W08 LiDAR 契约与假设（ADR-012）

模式固定为 `compensated`：trunk 刚性安装的多束几何传感器；不是理想水平射线。安装平移 (0.10,0,0.20) m、旋转 identity，RSP 发布 trunk→lidar_link。无新增质量/碰撞体；排除整个机器人，因而也忽略机器人对外界的遮挡，不能声称真实硬件视场。

每 5 个 0.02 s 策略步采一次，首次 t=0.10 s，单帧所有射线同时采样。360 方位角 [-π,π)，9 层仰角 [-20,-15,-10,-5,0,5,10,15,20]°；range_min=0.15 m，最大三维射程与扫描 range_max=12 m；scan_time=0.10 s，time_increment=0。噪声/丢点默认 0，可由启动只读参数配置，独立 RNG 固定 seed；不改变策略 RNG。

在物理步完成后于 owner 线程调用 mj_kinematics，再从 trunk xpos/xmat 计算原点与方向；调用已安装 MuJoCo 3.15.0 mj_multiRay（含 normal=None 参数）。机器人子树全部 geoms 改为保留 group 5，掩码仅排除 group 5；如外界已占 group 5 则拒绝启动，不改 contype/conaffinity/质量/摩擦。static=true、bodyexclude=-1，地面/墙/未来 mocap 障碍可见。只读射线测量，不把场景列表转成 scan。

命中点利用实际姿态变到 odom，过滤地面相对高度 z∈[0.15,1.20]m，再转换到 laser_stabilized（原点与真实 lidar_link 一致，旋转只有 trunk yaw）。距离取 xy 平面范数，按最近的 1°角格聚合最小距离，不插补、不补造缺失点。角格量化误差 ≤0.5°，最大横向误差 r·sin(0.5°)；多高度投影不等于单个物理水平截面。TF base_footprint→laser_stabilized 为随躯干姿态/高度变化的动态平移、identity旋转，50 Hz，与状态同 stamp。

+inf 只代表实际未命中射线所在角格；未覆盖、只有高度过滤回波或丢点的角格为 NaN（未知），不是空闲；近于 range_min 的有效高度回波标 -inf 并令内部 sensor_healthy=false，下一策略步命令置零继续站立采样。有效角格不足 75% 同样关门；恢复后允许新鲜命令。生成异常/非法射线结果不发布该帧，FAULT sensor_exception，ERROR diagnostics，清零命令并停止推进，需复位/重启。W10 必须结合未知区/近端盲区处理安全覆盖，不能把无回波当作无条件安全。

原始 PointCloud2 可选，lidar_link 坐标，只包含量程内实际命中点（含地面/高物，供检查过滤）；无逐束时间模拟，无材质/多径/强度/光束散度模型。姿态、安装外参和里程计均为仿真真值，无估计误差；地面限 z=0。

预注册验收：1m/3m中央束与角落/细柱解析误差 ≤0.02m；站立/前进/实际原地旋转90°后，投影扫描 odom 点到校准场墙面的 P95≤0.08m、最大≤0.12m（12m范围0.5°量化界0.105m），同时报告未量化点误差；所有机器人命中数=0、地面/高物过滤专例、手动移动外部 mocap 对象可见；采样 P95<50ms；ROS 10Hz仿真发布、同stamp TF、暂停无新scan、复位单调；RViz实际图像。

API依据：本地绑定 __doc__ 与 mujoco/include/mujoco/mujoco.h；[MuJoCo官方射线说明](https://mujoco.readthedocs.io/en/latest/APIreference/APIfunctions.html#ray-casting)仅佐证位置缓存要求，调用签名以本机3.15.0为准。

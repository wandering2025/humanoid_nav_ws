# W04 上游适配与基线

冻结 booster_deploy `7bb1462e2ac742eba5c01659b815e2e99fbe617f`，assets `12a97516f57469078707afb587a5f84c1cb110d1`；不修改上游。

复用 MujocoController 构造、BaseController.start/policy_step/stop、LocomotionPolicy 的观测/历史/动作映射。策略持有 controller 和 robot 引用；reset 清空 obs_history、last_action 和 filtered_dof_target，因此必须调用原 start/reset，不能只重置 MjData。

本地覆盖 update_state（按名称地址，root_lin_vel_b 旋转到机体系；此量不在官方策略观测中）、ctrl_step（同一 float32 PD/限幅，按执行器地址）、update_vel_command（不读 stdin），自行编排主循环。构造前先编译并验证模型，避免上游 qpos 全长初始化在多余自由度下先崩溃。原始切片/构造前提 nq/nv/nu=30/29/23，各名称、地址、执行器和 gear 强断言。

`baseline.py` 以相同初态和策略步序列，复用原 update_state/policy_step/ctrl_step，仅关闭 stdin、viewer 和固定 sleep。5 s 零命令后 10 s u_pol.x=0.5，计时只计控制/物理，不计加载。不是官方 GUI 墙钟性能测量。原始轨迹忽略保存 run_data/w04_baseline/trajectory.npy，轻量参数/索引/结果为 c1-baseline.json。命令在步首写入以对齐观测；官方 stdin 本来在 ctrl_step 读入会晚一拍，新运行器明确消除此 I/O 延迟。

physics_dt=0.002 s，decimation=10，policy_dt=0.02 s；PD 默认姿态/增益/限矩和 23 关节、21 策略关节顺序完整保存在 JSON。保留策略内部角速度、投影重力和关节观测，不借 W04 修正训练约定。

GUI 使用独立 MjModel/MjData 快照，不让 viewer UI 成为物理模型或数据的写入者。读取安装版 MuJoCo 3.15 viewer.py：sync 内部加锁，launch_passive 创建 daemon `_launch_internal` 线程，close 只请求 sim.exit。W04 先复制快照（viewer.lock）再 sync(state_only=True)；退出时 close 并 join 仅本次创建的 renderer 线程。初次复现 SIGSEGV(-11)，修复后真窗口截图+SIGTERM exit 0；原失败独立保存。

有限性检查覆盖物理前后状态和目标动作；每物理子步检查 MuJoCo bad-qpos/qvel/qacc 警告及时间倒退，防止内部自动 reset 把坏状态变成有限状态后掩盖故障。正式 reset 只在请求边界修改初始 qpos，行走从不编辑基座位姿。

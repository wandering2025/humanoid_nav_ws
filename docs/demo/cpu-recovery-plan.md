# W14 CPU 隔离续作方案（2026-10-15）

本次续作起始 HEAD `2f665a561ae53be63e3b660eae33eec287b4060b`，分支
`release/w13-formal-v1-demo`，起始工作区干净。用户要求继续W14，以电脑安全为前提。

宿主双显卡TDR根因尚未解除。用户的续作请求不能被解读为可以承诺GPU零风险；
默认图形路径保持封锁。本次先做CPU隔离，**不切卡、不启动软件图形窗口、不调用CUDA探测**。
这不满足原双窗口/两次默认命令验收，不能因此标W14_DEMO_READY。

选用内核Landlock（本机ABI7，要求至少ABI5），在任何ROS/Torch/MuJoCo导入前给本次进程
及其所有子进程施加不可撤销的GPU设备访问限制：不授予/dev父目录read/write/ioctl权限，
只允许null/zero/random/urandom/tty/pts/shm等运行必需项；不授予/mnt等宿主别名访问。
拒绝不明继承设备fd，启用no_new_privs。仅在无害/dev/full上验证EACCES，绝不为测试而打开/dev/dxg。
子进程exec继承拒绝规则；隔离失败就退出，不降级。未改变任何全局设备权限、挂载或系统设置。

清空本次子进程DISPLAY/WAYLAND_DISPLAY/WSL_INTEROP和CUDA可见设备；这不是清空PYTHONPATH。
原官方策略本来就在CPU（冻结源码RobotData.device=cpu），不改变策略/控制/导航/安全/物理参数。
本次禁用所有渲染；nice10和单线程库设置只降低本进程资源优先级，保护桌面响应。

阶段：无害隔离测试→CPU预检→修复并验证编排清理→CPU-only完整固定U01。
这些是W14辅助证据，不能冒充原两次默认GUI排练。若出现导航本身失败，保留并停止物理运行。

只读离线调查：Windows WATCHDOG目录权限拒绝，本机未找到cdb/WinDbg；未提升Windows管理员权限、
未安装工具或改执行策略。现有Windows WER与journal证据保持，最初驱动缺陷仍未定位。

参考：[Linux Landlock设备限制](https://www.kernel.org/doc/html/latest/userspace-api/landlock.html)。
限制只证明本次Linux进程的直接设备访问被约束，不是整机不会发生其他故障的保证；
也不能保护一个尚未启动、在本隔离边界外运行的WSLg宿主合成器，因此GUI仍封锁。

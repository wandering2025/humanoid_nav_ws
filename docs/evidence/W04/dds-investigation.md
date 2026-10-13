# W04 本机 DDS 调查

2026-10-13 UTC，本机 ROS Jazzy / FastDDS 2.14.7 / rmw_fastrtps_cpp 8.4.4。W03 验证的是单进程 run/launch，没有验证跨进程话题。

默认 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST：实际 runner 自身 READY/clock/trace 正常，验收客户端无状态消息；加端点诊断发现父进程仅见 UNKNOWN 子发布端点，子进程无父订阅。controls-failed-discovery/endpoints/udp.json 保存失败，子进程均只由本次驱动 SIGTERM 清理。

逐项缩小实验（每次都保留前一现象、增加新诊断）：
- 去掉机器人业务，用 std_msgs/String 两进程最小探针：同样失败。
- UDPv4 环境变量、显式仅 loopback UDP XML、legacy localhost、静态 peer、SYSTEM_DEFAULT 与显式 locator：未解决。
- 系统 Python 与 venv 同样失败；安装版 C++ talker/listener 4 条发布、0 条接收。排除 Python/业务代码。
- 同进程双 ROS 节点 2 s 收 19475 条；两进程普通 UDP 回环、模拟 RTPS 前缀固定端口25910/27910双向正常。排除一般 UDP 完全不可用，不能据此保证 DDS 发现正常。
- 本地 UDP discovery server 未解决；已有 FastDDS TCP discovery server（127.0.0.1:42114，server-id=0）成功：父/子互见名字及订阅，6 s 探针收到99条。

结论：跨 participant 默认 UDP 发现路径有本机故障；底层根因未定。选择显式可复现 TCP discovery 路径，不改系统网络、依赖和冻结上游。ADR-009 记录此本地环境增补。server 是发现基础设施，数据仍走 middleware 协商的传输；不是 ROS clock/命令生产者，不代替实际 ROS 订阅验收。

用法：每个终端 `export T1_LOCAL_DDS=1` 后 `source scripts/env.sh`；单独一个终端 `scripts/start_local_dds.sh` 前台运行；其余终端运行 ROS 命令。测试完在 server 终端 Ctrl+C。端口被占用会报错，不停止其他 server；需要独立系统时所有终端统一设 T1_DDS_PORT。Discovery server 模式的隔离应使用独立端口，不能只依赖 ROS_DOMAIN_ID。

官方依据（仅参考其配置能力；实际结论来自本机探针）：[FastDDS 2.14.7 环境变量](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/env_vars/env_vars.html)；[Jazzy participant.cpp](https://github.com/ros2/rmw_fastrtps/blob/jazzy/rmw_fastrtps_shared_cpp/src/participant.cpp) 的 LOCALHOST 分支额外创建 SHM/UDP transports，说明为何仅指定 built-in UDP 环境变量不足以隔离。

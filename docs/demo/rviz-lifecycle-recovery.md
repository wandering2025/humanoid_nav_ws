# W14 RViz退出修复：离线假设与严格复验预登记

本轮起始2026-10-15，HEAD `f5f401165270750b777630ad80c3010dd816462b`，release/w13-formal-v1-demo，clean。
用户新授权严格限定为RViz代码/配置/信号/等待/进程生命周期修复；不改导航、Nav2参数、安全区、物理碰撞、动态轨迹、起终点、seed、策略。
禁止驱动/BIOS/注册表/显卡模式/Windows显示改动。保留W13 FAILED、黑屏、CPU失败和所有RViz失败；不隐藏stderr/−6。

离线事实：最新失败在收到SIGINT后报告corrupted double-linked list，SIGABRT−6。RViz运行期间已显示并完成导航。
本机RViz14.1.24、Qt5.15.13、Mesa25.2.8；core_pattern由wsl-capture-crash接管，无此RViz可用core，未安装gdb或调试包。
因此不能精确证明堆中哪个对象被破坏，也不能排除DDS/渲染库等其他因素。

最可信的可检验假设是退出生命周期竞争：标准RViz的rclcpp信号处理先撤销ROS context，而VisualizerApp析构仍先ROS shutdown再销毁frame/display。
[Jazzy源码](https://github.com/ros2/rviz/blob/jazzy/rviz_common/src/rviz_common/visualizer_app.cpp)及
[上游退出顺序修复提案](https://github.com/ros2/rviz/pull/1881)支持该机制的存在，但该提案尚不能证明本机同源。
后者同时描述批量移除display的Qt模型嵌套删除问题。当前配置无Camera，不把该问题当成已证根因。

最小修复是仓库内约百行RViz薄宿主，链接已安装库，不重编/替换/升级ROS或驱动：
- rclcpp初始化禁用默认信号处理；SIGINT/TERM仅写sig_atomic_t，不触碰Qt/ROS堆对象。
- Qt主线程定时器接收退出请求，事件循环结束后停止更新。
- 逐个takeDisplay/delete，ROS context此时仍有效，插件库仍随frame保留。
- 然后ROS shutdown，最后销毁frame/node，正常析构真正运行；不使用_exit、exit码重写或忽略stderr。
- 实际OGRE GL_RENDERER必须包含NVIDIA RTX4050，强设本进程xcb；RViz显示15Hz，仅显示负载参数，原Nav2/物理/策略不变。
- GPU/Windows事件/新增WSLg日志与Qt心跳监控；清理异常、−6或SIGKILL均不得判成功。

编译产物留ignored build/demo-rviz；scripts/demo/build_rviz.sh仅用已有cmake/g++和安装库，源hash匹配才复用。
默认脚本可自动编译，不要求用户另开环境。原CPU RViz程序保留作历史，不在失败后切换回它重试。

严格复验顺序：所有非图形测试+编译/静态检查通过→只允许一次最小RViz启动/正常关闭→若成功，再预登记同一干净SHA两次默认命令。
最小测试仅RViz NVIDIA、DDS与保存的地图显示，无MuJoCo/导航/机器人物理，不能冒充完整演示。
完整组合前MuJoCo沿已通过的最小GL/小窗/官方静态T1证据，RViz须通过本次唯一测试；逐个启动。
每轮之间清理全部登记PID、端口释放、Windows和GPU检查；不得把不同版本成功拼成两轮。

任一出现Removing Device、DX12_PAGEFAULT、LiveKernelEvent141/显示超时、RViz−6、持续无响应、异常显存、宿主黑屏/鼠标消失或图形进程需SIGKILL：
停止扩载并终止最近任务，保存证据，不再重试、不换路径继续。最小失败则完整两次NOT_RUN、W14_BLOCKED。
本次授权不等于整机GPU安全保证；W15不开始。视频仍MISSING，人工步骤保留。

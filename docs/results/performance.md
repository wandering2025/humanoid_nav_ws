# W13 性能测量

硬件：Intel Core Ultra 7 155H，WSL2 Ubuntu 24.04，22逻辑CPU；ROS2 Jazzy、MuJoCo3.15.0，具体Python/Torch/解释器、线程、checkpoint和命令行见逐run manifest。torch.set_num_threads(1)，同一进程owner推进2ms物理/20ms策略，9×360束扫描10Hz，Nav2 controller20Hz/MPPI1000×56，visualize=false、无ghost。

正式40例采用target_rtf=1节拍。独立性能campaign使用headless target_rtf=1.1、GUI target_rtf=1.0，仅测该节拍是否可达，不声称极限容量；不混入正式可靠率。每模式只有一例S04，分布样本为该运行内策略/扫描/CPU窗口。GUI含实际MuJoCo passive viewer 30Hz和RViz；截图随专用campaign保存。

正式headless RTF严格目标按未舍入值比较。target_rtf=1的节拍上限使观测值接近1但有窗口抖动；没有四舍五入判PASS。额外headless1.1运行因FollowPath ACK超时失败，RTF不能证明完整导航>1.0；GUI1.0运行因终点未收敛120s超时，真实窗口/耗时仍保留，不计正式可靠率。

| 运行组 | 指标 | N | 中位数 | P5 | P95 | 最大 |
|---|---|---:|---:|---:|---:|---:|
| formal_headless_capped | rtf  | 133335 | 0.99999986 | 0.95104858 | 1.00001013 | 1.00037989 |
| formal_headless_capped | step_compute_ms ms | 152380 | 2.13274250 | 1.66634270 | 4.44119085 | 51.43742100 |
| formal_headless_capped | policy_compute_ms ms | 152380 | 0.71400500 | 0.60018500 | 1.62839285 | 48.64823500 |
| formal_headless_capped | physics_compute_ms ms | 152380 | 1.00721950 | 0.76449570 | 2.39082105 | 29.79814400 |
| formal_headless_capped | scan_compute_ms ms | 30460 | 1.38795250 | 0.98720265 | 3.30754700 | 22.73450800 |
| formal_headless_capped | bt_navigator CPU ms/wall-s | 3800 | 99.973 | 79.342 | 179.874 | 318.672 |
| formal_headless_capped | behavior_server CPU ms/wall-s | 3800 | 59.960 | 39.773 | 99.920 | 159.815 |
| formal_headless_capped | planner_server CPU ms/wall-s | 3800 | 79.865 | 59.482 | 119.960 | 219.548 |
| formal_headless_capped | controller_server CPU ms/wall-s | 3800 | 139.881 | 79.393 | 239.984 | 477.810 |
| formal_headless_capped | collision_monitor CPU ms/wall-s | 3800 | 79.976 | 59.828 | 139.878 | 239.888 |
| formal_headless_capped | amcl CPU ms/wall-s | 3800 | 79.999 | 59.500 | 159.891 | 359.481 |
| formal_headless_capped | t1_sim CPU ms/wall-s | 3800 | 279.994 | 239.829 | 539.290 | 1177.576 |
| performance-headless-v1 | rtf  | 849 | 1.08720218 | 1.08069069 | 1.09999892 | 1.10000865 |
| performance-headless-v1 | step_compute_ms ms | 1396 | 1.83281450 | 1.62474600 | 2.58082100 | 26.27613000 |
| performance-headless-v1 | policy_compute_ms ms | 1396 | 0.67936650 | 0.60759325 | 1.20902650 | 24.79752200 |
| performance-headless-v1 | physics_compute_ms ms | 1396 | 0.82180150 | 0.72658325 | 1.20305625 | 3.32536600 |
| performance-headless-v1 | scan_compute_ms ms | 279 | 1.09066800 | 0.96853690 | 1.82568810 | 3.78797700 |
| performance-headless-v1 | bt_navigator CPU ms/wall-s | 1 | 99.556 | 99.556 | 99.556 | 99.556 |
| performance-headless-v1 | behavior_server CPU ms/wall-s | 1 | 59.734 | 59.734 | 59.734 | 59.734 |
| performance-headless-v1 | planner_server CPU ms/wall-s | 1 | 79.645 | 79.645 | 79.645 | 79.645 |
| performance-headless-v1 | controller_server CPU ms/wall-s | 1 | 79.645 | 79.645 | 79.645 | 79.645 |
| performance-headless-v1 | collision_monitor CPU ms/wall-s | 1 | 59.734 | 59.734 | 59.734 | 59.734 |
| performance-headless-v1 | amcl CPU ms/wall-s | 1 | 59.734 | 59.734 | 59.734 | 59.734 |
| performance-headless-v1 | t1_sim CPU ms/wall-s | 1 | 258.847 | 258.847 | 258.847 | 258.847 |
| performance-gui-v1 | rtf  | 6788 | 0.99911098 | 0.94932410 | 1.00000740 | 1.00006810 |
| performance-gui-v1 | step_compute_ms ms | 7263 | 4.20109200 | 2.05661400 | 10.34200500 | 32.82605500 |
| performance-gui-v1 | policy_compute_ms ms | 7263 | 1.16480300 | 0.72443280 | 3.36317750 | 27.30253000 |
| performance-gui-v1 | physics_compute_ms ms | 7263 | 2.30954200 | 0.86409620 | 6.85748560 | 29.35927000 |
| performance-gui-v1 | scan_compute_ms ms | 1452 | 2.55550400 | 1.21960375 | 7.63582165 | 19.76835500 |
| performance-gui-v1 | rviz2 CPU ms/wall-s | 242 | 578.100 | 417.411 | 1055.587 | 1117.319 |
| performance-gui-v1 | bt_navigator CPU ms/wall-s | 242 | 159.858 | 119.337 | 279.133 | 319.769 |
| performance-gui-v1 | behavior_server CPU ms/wall-s | 242 | 79.967 | 59.613 | 139.851 | 159.931 |
| performance-gui-v1 | planner_server CPU ms/wall-s | 242 | 99.981 | 59.959 | 179.595 | 199.954 |
| performance-gui-v1 | controller_server CPU ms/wall-s | 242 | 199.829 | 139.590 | 418.253 | 558.862 |
| performance-gui-v1 | collision_monitor CPU ms/wall-s | 242 | 119.998 | 99.271 | 239.394 | 259.225 |
| performance-gui-v1 | amcl CPU ms/wall-s | 242 | 119.922 | 59.936 | 259.715 | 319.970 |
| performance-gui-v1 | t1_sim CPU ms/wall-s | 242 | 1259.616 | 1054.173 | 1785.311 | 1975.260 |

RTF来自10s墙钟滑窗Δsim/Δwall，排除前10s墙钟启动段；其他耗时为所有策略步的墙钟计时，scan仅统计due样本。physics_compute_ms包含10个PD/物理子步、mocap及接触钩子；policy_compute_ms只包policy_step；step_compute_ms还含状态处理。

导航耗时为Linux /proc 对自持进程约0.5s墙钟窗口采样的user+sys CPU增量，归一化为CPU ms/wall-s；controller_server包含costmap和控制线程，不能解释为单次控制回调延迟。仅统计首个目标接受至工具结果结束之间的活动任务样本。按进程区分planner/controller/BT/behavior/AMCL/CM，完整采样及原trace留ignored raw并保存SHA。

优化选择：现有配置已关闭MPPI采样轨迹visualize与ghost，raycast已批处理。本步增加观测并测量，未改变策略、物理dt/decimation、安全参数、扫描分辨率或Nav2参数。没有为改善数字改变/clock；若GUI未达0.7或headless未达1.0如实标记。

正式headless：中位数0.9999998608，严格目标≥1.0：FAIL。

专用运行结果：FAIL_PLANNING，run_id=20261014T231432Z_performance-headless-v1_S04_s1003_e642e6。

performance-headless-v1: 中位数1.087202，目标≥1.0：PASS。

专用运行结果：FAIL_TIMEOUT，run_id=20261014T231504Z_performance-gui-v1_S04_s1003_5ad1d5。

performance-gui-v1: 中位数0.999111，目标≥0.7：PASS。

# Presentation talk track

Use these lines for **W16 exploration** (`run_exploration_demo.sh`). The English lines are for presenting; the Chinese lines are rehearsal notes. Pause while the robot moves. The older backup video shows guided waypoints, so it needs the legacy explanation at the end.

| Stage | English | 中文 |
|---|---|---|
| A and prior knowledge | This is Booster T1 in MuJoCo. Its task specifies only B relative to A. The navigation system starts without a map of the walls or passages. | 这是MuJoCo中的Booster T1。任务只给出B相对A的坐标，导航系统起初不知道墙和通道。 |
| Observing and deciding | LiDAR builds the map as the robot walks. The explorer chooses where to observe next from the boundary between known free space and unknown space. | 机器人边走边用LiDAR建图。探索器根据已知自由区域与未知区域的边界，自主选择下一个观察位置。 |
| Planning | Blue is the latest Nav2 plan, orange is the selected target, and purple is the travelled path. The robot may first move away from B to find a passage around a wall. | 蓝线是最新Nav2规划，橙色是当前目标，紫线是已走轨迹。为找到绕墙通道，机器人可能先远离B。 |
| Dynamic obstacle | The moving obstacle has real collision geometry and appears in LiDAR. When it blocks the passage, the robot waits while continuing to observe. | 移动障碍有真实碰撞几何，也会进入LiDAR观测。通路被阻挡时，机器人等待并继续观察。 |
| Continuing toward B | The obstacle has withdrawn. The robot continues when its live observations provide a safe route. It heads to B once a known route becomes available. | 障碍撤离后，实时观测确认存在安全路线，机器人继续行走；发现通往B的已知路线后就前往B。 |
| Arrival and result | The robot has reached B and stopped. After cleanup, the terminal reports the arrival, contact, fall and shutdown checks. | 机器人到达B并站稳。清理结束后，终端报告到达、接触、跌倒和退出检查结果。 |

## Technical choices

| English | 中文 |
|---|---|
| I use SLAM Toolbox for mapping, NavFn for global planning and Regulated Pure Pursuit for path tracking. The existing safety chain filters motion commands. | 使用SLAM Toolbox建图、NavFn全局规划、Regulated Pure Pursuit跟踪路径，现有安全链过滤运动指令。 |
| Observation goals are selected online from reachable known free space. The explorer receives no scene geometry, preset waypoints or obstacle trajectory. | 观察目标从已知可达自由区域在线选出。探索器不接收场景几何、预设航点或障碍轨迹。 |
| Walking comes from the existing policy and MuJoCo contact dynamics. The planner must respect the walking and safety constraints. | 行走由既有策略与MuJoCo接触动力学产生，规划必须满足步态和安全约束。 |
| SLAM stays active throughout. A filter uses changes in observed scans to suppress transient returns in the mapping branch; navigation and the safety monitor still receive the full raw scan. | 全程持续SLAM。过滤器根据扫描观测变化抑制建图分支中的瞬态回波；导航与安全监控仍接收完整原始扫描。 |
| This simulation uses ideal odometry and truth attitude compensation. The moving obstacle is collidable but kinematic and cannot be pushed. | 仿真使用理想里程计和真值姿态补偿。移动障碍参与碰撞检测，但属于不可被推动的运动学实体。 |
| The task file gives B in metres: forward and left relative to the robot at A. I can change it between runs; reachability is discovered online. | 任务文件以米给出B相对A的前向、左向偏移。可在运行之间修改，能否到达由在线探索发现。 |
| A time limit or lack of a safe route leads to a recorded failure and safe stop. These demonstrations do not establish success in every unknown environment. | 超时或没有安全路线会记录失败并安全停车；这些演示不证明任意未知环境中都能成功。 |

Do not promise a fixed route, waiting time or final error: read this run's results. Say “planning wait” unless logs confirm a Collision Monitor protective stop. During `--hold`, cleanup is still pending; only report clean shutdown after the final terminal result. An RViz label can retain an earlier rendered state, so use the physical settling and final result to confirm arrival. If interrupted before arrival, say “The stop path was exercised; this run was canceled.” / 如到达前中断，可说：“本次验证了停车路径，任务已取消。”

## Legacy video explanation

“This earlier recording follows preset observation waypoints. It uses an absolute map goal at B=(12,0), with SLAM corrections paused during dynamic navigation while live LiDAR obstacle avoidance continues. The current W16 live demo chooses observation targets online and keeps SLAM active.”

“这段旧录像采用预设观测航点和map系绝对目标B=(12,0)。动态导航时暂停SLAM修正，实时LiDAR避障继续工作。当前W16现场演示在线选择观察目标，并保持SLAM运行。”

# Presentation talk track

Use the English lines when presenting. The Chinese lines are rehearsal notes. Each row corresponds to a visible stage; pause while the robot moves.

| Stage | English | 中文 |
|---|---|---|
| A and initial map | This is Booster T1 in MuJoCo. It starts at A with a new SLAM session and no complete map. | 这是MuJoCo中的Booster T1。机器人从A点开始，SLAM会话是新的，还没有完整地图。 |
| Moving and mapping | The robot follows an autonomous mapping route. LiDAR builds the map as it moves through the rooms and turns around the partition. | 机器人自主执行建图路线，用LiDAR逐步建立地图，并通过多个区域、绕过隔墙。 |
| Planning | Blue shows the current Nav2 path. Cyan shows the full A-to-B route on the acquired map. Purple shows where the robot has walked. | 蓝线是当前Nav2路径，青线是所得地图上的A到B概览，紫线是实际已走轨迹。 |
| Dynamic obstacle | The orange cart enters the passage. The robot detects it through LiDAR and waits for a safe route. | 橙色障碍进入通路。机器人通过LiDAR感知它，等待安全通路恢复。 |
| Continuing | The cart has cleared the route. The robot plans again and continues to B. | 障碍已经撤离，机器人重新规划并继续前往B。 |
| Arrival | The robot has reached B and stopped. The final checks confirm arrival, zero contacts, no falls and clean shutdown. | 机器人到达B并停车。最终检查确认到达、零接触、无跌倒及正常清理。 |

## Technical choices

| English | 中文 |
|---|---|
| I use SLAM Toolbox for mapping, NavFn for global planning and Regulated Pure Pursuit for path tracking. The existing safety chain filters motion commands. | 使用SLAM Toolbox建图、NavFn全局规划、Regulated Pure Pursuit跟踪路径，现有安全链过滤运动指令。 |
| The mapping route is guided by waypoints. It is not frontier exploration. The motion itself is autonomous. | 建图采用预设航点引导，不是frontier exploration；运动过程自主执行。 |
| The main challenge was matching navigation to a walking policy with limited low-speed control. Continuous mapping passes and direct path tracking reduced redundant motion. | 主要挑战是导航与低速控制受限的步行策略配合。连续通过观测点和直接路径跟踪减少了冗余运动。 |
| This simulation uses ideal odometry and truth attitude compensation. During dynamic navigation, SLAM corrections pause; LiDAR costmaps and obstacle avoidance remain live. | 仿真使用理想里程计和真值姿态补偿。动态导航期间暂停SLAM修正，LiDAR代价地图和避障仍实时运行。 |
| The B-point file is read at each launch. I can change the destination within the supported area without changing code. | 每次启动都会读取B点配置文件。在支持区域内可以直接修改目标，无需修改代码。 |

Do not promise a fixed waiting time or final error: read this run's results. Say “planning wait” unless the logs confirm a Collision Monitor protective stop. If interrupted, say “The stop path was exercised; this run was canceled.” / 如中断，可说：“本次验证了停车路径，任务已取消。”

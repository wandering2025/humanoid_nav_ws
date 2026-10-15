# Advanced SLAM and Navigation Demo v0.2 发布清单

**ADVANCED_DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**

唯一推荐启动命令：

```bash
/home/haoyang/humanoid_nav_ws_advanced/scripts/run_advanced_demo.sh
```

| 项目 | 冻结值 |
|---|---|
| 分支 | feature/advanced-slam-demo-v0.2 |
| worktree | /home/haoyang/humanoid_nav_ws_advanced |
| 录像实测实现 SHA | `f9286749d8f6c78122df6e9e25a50cff21977d5b` |
| 最后代码变更 SHA | `ef61e722f0466d36556807e36ddf458e6fe75c6d` |
| 最终证据 SHA | `8f091b2fc438e2a2845fa6c42819c820d7ed36d5` |
| 发布 tag | `demo-v0.2.0`，指向完成文档/交接后的提交；源码与录像实测实现一致 |
| 完整运行 ID | `20261015T180213Z_advanced_c20b40f2-12f5-4b5c-8c96-0d5a68461b8f` |
| 结果 | SUCCESS，所有基础组件和目标工具退出0，未使用 DDS 关闭例外 |
| 稳定兜底 | demo-v0.1.0，`0f51d54b1dbfdde0664966118b5546109a3f6ff2` |
| 原稳定 tag object | `844036e69644272dce6178552dd10778e80d4d35` |

[最终完整录像](../evidence/advanced/raw/final-recording-02/advanced-demo.mp4)：**368.25秒（6分8.25秒）**，2048×852、4fps H264/avc1、无音频、14,703,489 bytes。前360.25秒保留同一次运行的真实单调时钟时间，最后8秒为明确标记的运行后证据卡。1139组真实双窗口抓取；1473/1473编码帧软件解码通过；13个关键画面视觉审查通过。最大采样间隔0.725秒、两窗口最大顺序读取时间差0.339秒；没有加速、拼接其他运行或用合成画面替代运动。

SHA256：`3f9547f14e7ecadd4d0f115c4a562291dc4530e4de7a95d7633041d0f7a3716a`

录像章节：0:00 A与初始地图；0:26西区地图增长；1:06北侧宽廊；1:55东区；4:14返回A；4:33清楚显示绕墙路径；5:09障碍接近；5:15障碍停留与绕行；5:26撤离后继续；5:59 B与停稳；6:00.25运行后证据卡。

## 实际结果

采用 **guided autonomous mapping**，9个预定义航点由 Nav2 自主执行，无键盘遥控、无先验地图、无 frontier exploration 声称。SLAM Toolbox 全程在线、独占 map→odom；地图保存自实际 `/map`，不由场景几何生成。

西区—7.5m宽北廊—东区组成三个空间区域，隔墙遮挡 A(0,0) 与 B(14,0)。建图后返回A，再绕墙到B。原目标工具保留分段接近：首条全局路径19.7768m，终点为B前方(14,1.5)，随后完成到B的最后接近；不是14m空旷直线导航。最终实际位置误差0.16184m、朝向误差5.5315°，通过原有到达及停稳判据。

地图已知面积由2.8525m²增长到347.055m²（含占据栅格）。离线检查的西区、东区和北廊区域均完全被观测；所有检查墙面/隔墙线到占据栅格的距离最大0.0368m。检查区域和方法见[地图覆盖证据](../evidence/advanced/final-map-coverage.json)；这不代表真实传感器定位精度。

ADV01箱体以0.6m/s接近、停留6秒并撤离。实际响应为 **REPLAN_AND_CONTINUE**，没有保护等待：152帧活跃期扫描命中、23帧costmap标记；规划偏移0.81175m，机器人实际偏移0.76598m，活跃期行进5.1805m。随后到达B。

静态障碍接触0、动态障碍接触0、非足接地0、跌倒0。最近机体几何间距1.82674m，为10Hz采样最小值，非连续下界。安全门锁存停车ACK、物理静止、SLAM deactivate/cleanup确认、所有组件退出0、无残留、端口42127释放全部通过。GPU峰值64°C、1354MiB/6141MiB、61%利用率，无新图形监控错误。

## 证据与回归

- [最终运行](../evidence/advanced/final-run.json)、[视频与视觉审查](../evidence/advanced/final-video.json)、[源码/稳定版本保护](../evidence/advanced/source-audit.json)、[原始文件哈希索引](../evidence/advanced/raw-index.json)。索引覆盖16个高级运行目录、965个文件，逐帧PNG由两个已核验的frames.jsonl绑定；原始数据、构建输出和视频保持ignored。
- 最新CPU192项通过，1项仅适用于v0.1“无新模式”的历史源码断言未用于高级分支；该断言在原稳定分支完整186项测试中通过。四个ROS包用system colcon构建通过，未安装/升级依赖。
- [中断负测04](../evidence/advanced/interrupt-04.json)：行进中一次Ctrl+C，目标CANCELED/exit2；基础组件全部0，停车/物理静止/清理/端口通过。主动取消不被标成到达SUCCESS。
- [v0.1回归](../evidence/advanced/stable-v01-regression.json)：原入口、原工作区固定U01/seed4000完整headless运行SUCCESS，四项零接触/跌倒、停车、全部组件0和端口通过。稳定分支、tag、原入口及外部上游均未改动；没有重跑W13研究，没有push。
- 所有失败证据保持原样，包括初始未知区域规划失败、等待专用判据拒绝实际绕行、观察器接口/信号竞态、旧墙钟心跳误报、目标工具关闭及DDS−11、录像01的SLAM−6。修复记录见[handoff](handoff.md)。录像01的1354组原始帧保留，未伪装为发布视频。

## 已知限制与复现

理想真值里程计、真值姿态补偿、模拟LiDAR、平地、固定航点/场景/种子、mocap障碍和响应式避障均保留。终点朝向收敛时间随步态死区与回弹变化。WSLg/NVIDIA OpenGL底层仍使用D3D12；监控不能保证故障隔离。外部DDS历史关闭风险未宣称根治。W13仍为 **FAILED/36/40**，本发布不是可靠性或硬件安全认证。详见[架构与限制](architecture.md)。

[操作说明](quick-start.md) · [讲解提纲](presentation-outline.md) · [14项验收](final-review.md) · [独立录像流程](recording.md)

# W11R1V 独立验证结果

最终状态：**INDEPENDENT_VERIFICATION_FAIL**。

冻结审计、4包构建和151项测试通过。第1次冷启动就绪通过，但正常关闭时 `navigation_starter` 在 `node.destroy_node()` / `client.destroy_when_not_in_use()` 抛出未捕获 `KeyboardInterrupt`，实际退出码−2。冻结矩阵要求冷启动无异常退出，因此候选FAIL，按停止规则不再执行后续37次物理运行，不以重跑寻找成功。

- REPAIR_SHA：`47a383bdeb57f9531485a464dec22d7b2514cdb3`。
- HANDOFF_SHA / 开始HEAD：`284aba1e39c239063b5ca92725fde951bf31cc72`；初始clean。
- 预注册：`8cd6adc0af387f2101168c278157fdc87ee8e22d`；构建/测试针对该HEAD。
- 阶段A证据：`636386e00c0beacbe245f154fd9aa80b9a3b2998`。
- 物理被测HEAD：`61264cfb27d32fb831079d2f85989a8d8eedb7aa`；只增加验证证据/记录，src/scripts与REPAIR一致。
- 源码/脚本指纹：`5ea0ccb65508bd4c70030759fb0d78aa7b382c49e2e303ca7858ea706117fe58`，开始/结束均匹配；formal-v1原1371文件哈希全同。

## 首次失败

run ID：`20261015T093701Z_w11r1-independent-v1-startup_01_s0`。

使用冻结W11 `startup_probe(1)`，只将导入的harness绑定为冻结现代 `evaluation.harness`，使输出进入新目录；未修改原函数或判据。新Python/ROS context、新DDS和新launch进程组。环境端口42124/domain34，RViz开启，seed0、static_world、起点(0,0,0)。原调用封装全文见 `startup-invocation.txt`。

14.219661墙钟秒 / 10.8仿真秒就绪：8个lifecycle节点实际响应active，initialpose一次，clock推进，108帧非空scan，FollowPath实际ACK及预期INVALID_PATH103结果，两costmap服务响应。空path探测的ABORTED103是预注册就绪探测，不是导航失败。未arm、未发送NavigateToPose目标，策略命令全零。

日志199行报告成功；200行是冻结harness在成功后按原入口关闭自身launch，202行向starter转发一次SIGINT；219–234行明确显示清理期间KeyboardInterrupt；300行记录PID99620退出−2。不是外部用户中断，没有外部进程占端口、错误命令或错误输出目录，不满足INVALID条件。wrapper/launch/DDS退出0不能覆盖子进程异常。

18个launch子进程：17个exit0、starter一个−2。没有SIGTERM/SIGKILL升级，所有自持PID已消失，42124/42126无监听；未关闭无关进程。故障限定于启动完成与正常关闭的集成边界，不推断未执行案例的导航行为。

## 完整矩阵状态

| 阶段 | 本次结果 |
| --- | --- |
| A | 4包build；151tests，0errors/0failures/0skipped |
| B 冷启动 | 完整通过0/10；执行1、FAIL1、未执行9；就绪1/1通过 |
| C | C01/C09/B04/B06各0/3执行，全部NOT_RUN |
| D | W11 S01–S05全部NOT_RUN |
| E | 取消、stop锁存、rearm及目标替换NOT_RUN |
| F | crossing/blocking/approach各0/3执行，S03未执行 |
| G | 冻结与进程最终审计完成；无遗留 |

唯一已执行物理运行环境碰撞0、障碍接触0、非足接地0、跌倒0。未执行案例不能据此宣称安全或到达。
FAIL run仅上述STARTUP-01；物理INVALID run无。首宿主审计误计审计器本身及系统unattended-upgrades，首离线解析器误匹配普通`signal_handler(SIGINT/SIGTERM)`信息，均原样披露；派生正确审计只依据真实退出行，FAIL仍成立，无物理重跑。

## 证据与范围

`verification-plan.yaml`预注册后未改；`initial-audit.json`、`host-initial-audit.json`与`final-freeze-audit.json`核对冻结。`test-result.txt`及两个XML保留完整自动化结果；`automated-test-index.json`列明各测试。协议故障测试含mock/合成输入，真实manager/planner/controller使用ROS但无机器人物理，不能代替未执行的外部运动核查。第二命令发布者/取消等物理故障行为未执行，不标PASS。

`startup-01-assessment.json`为最终逐项审计，`startup-01-closed.json`含manifest/参数地图checkpoint哈希/图/clock/scan/进程，`startup-01-excerpt.txt`保留带原行号的失败现场。首解析器审计保存在`startup-01-initial-parser-audit.json`；全部原始日志/轨迹保留于ignored `run_data/w11r1-independent-v1/startup/01/`和`run_data/20261015T093701Z_w11r1-independent-v1-startup_01_s0/`，路径、大小、SHA256见`raw-artifact-index.json`。

退回第一部分修复阶段：责任范围为W11 `navigation_tools.starter`成功输出至清理退出期间的SIGINT处理与冻结冷启动关闭契约，需修复者提供新候选SHA和新交接后重新独立验证。当前不满足重执行W13 formal-v2的条件。W13保持FAILED，W14/W15保持TODO。

本次没有修改源码、配置、参数、冻结测试入口、地图、场景、种子或事件；没有执行W13 formal-v2或RTF正式验收，没有启动W14/W15，没有委派、push、reset、rebase、clean或恢复用户文件。

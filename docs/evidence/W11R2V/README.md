# W11R2V 第二次独立验证

**INDEPENDENT_VERIFICATION_FAIL**。本轮从头执行，没有复用W11R1V或W11R2冒烟样本。

REPAIR_SHA `166b4a15835a68eb6fd7d1e685f0cc68421f7d77`；HANDOFF_SHA/开始HEAD `a4589cbdd65e826d2b9092278e68e21f2322189b`。
PHYSICAL_TEST_HEAD `53d8f0542c3891a1bfbc95b5eb7805dda1697be6`。源码JSON指纹 `4ffaee2ab9b6e31fdf2eb5920ce7ab27c5e78a8b4cbc616949f6ce616d71c28c` 与冻结候选匹配。

静态：4包build通过，164/164包测试、15/15既有评测器测试，0errors/0failures/0skipped。新增13关闭测试已重新执行：单次SIGINT正常exit0且无KeyboardInterrupt traceback、幂等/已关闭context清理、注入真实RuntimeError保持exit1和traceback。反向注入的预期exit1属于通过测试，不计物理异常。

## 首次有效失败与归属

第9次run `20261015T102107Z_w11r2-independent-v1-startup_09_s0_13f20b` 的既有采集断言 `dds_exit_zero` 为false；采集器退出1，因此本轮完整冷启动阶段FAIL。
starter与全部18个launch子进程exit0，launch父进程exit0；无traceback、SIGTERM/SIGKILL升级或残留。DDS PID124464在一次正常SIGINT后已结束，日志只显示Server shut down。冻结采集器仅保存returncode==0布尔值，未保存精确整数退出码；Session.close已wait完成，false表明非零，但整数不可恢复，不作猜测。

归属：**DDS测试支撑进程退出检查失败，根因未确定**；没有证据将其归因于本轮starter修复。机器人启动/关闭结果与DDS支撑进程结果分开保存。按预注册零退出门槛保留FAIL，不在看到行为结果后改为INVALID，不重跑替换，不修改采集器断言、源码、依赖或配置。停止第10次及后续导航阶段，退回修复agent核查DDS/session关闭边界和精确退出码记录缺口。

## 10次冷启动逐次结果

| 序号 | run ID | 就绪wall/sim秒 | starter/launch子进程 | DDS | 完整结果 |
| --- | --- | --- | --- | --- | --- |
| 01 | `20261015T101841Z_w11r2-independent-v1-startup_01_s0_92c2bb` | 13.828967/10.24 | 0；18/18 exit0 | exit0 | PASS |
| 02 | `20261015T101900Z_w11r2-independent-v1-startup_02_s0_371992` | 14.424747/12.02 | 0；18/18 exit0 | exit0 | PASS |
| 03 | `20261015T101916Z_w11r2-independent-v1-startup_03_s0_ca353a` | 14.305098/11.46 | 0；18/18 exit0 | exit0 | PASS |
| 04 | `20261015T101935Z_w11r2-independent-v1-startup_04_s0_e1f72a` | 14.286314/11.56 | 0；18/18 exit0 | exit0 | PASS |
| 05 | `20261015T101951Z_w11r2-independent-v1-startup_05_s0_c8e234` | 14.331220/11.32 | 0；18/18 exit0 | exit0 | PASS |
| 06 | `20261015T102010Z_w11r2-independent-v1-startup_06_s0_906359` | 15.337735/12.50 | 0；18/18 exit0 | exit0 | PASS |
| 07 | `20261015T102028Z_w11r2-independent-v1-startup_07_s0_69b60f` | 17.291942/13.18 | 0；18/18 exit0 | exit0 | PASS |
| 08 | `20261015T102050Z_w11r2-independent-v1-startup_08_s0_6319ce` | 15.025165/12.12 | 0；18/18 exit0 | exit0 | PASS |
| 09 | `20261015T102107Z_w11r2-independent-v1-startup_09_s0_13f20b` | 14.584681/11.50 | 0；18/18 exit0 | 非零，整数未保存 | FAIL |
| 10 | 无 | 未执行 | 未执行 | 未执行 | NOT_RUN |

实际完成9次物理运行，完整通过8/10、有效失败1、未执行1；9/9就绪，9/9 starter exit0，162/162 launch子进程exit0。所有已执行案例initialpose一次、clock推进、scan持续非空、8节点active、FollowPath真实ACK/103及两个costmap响应；未提前发送NavigateToPose目标、步态命令全零。每次全新Python/ROS context/DDS/launch，ready后只发送一次正常SIGINT，无延迟或重试。

## 后续矩阵状态

| 阶段 | 首次结果 |
| --- | --- |
| B 原失败案例 | C01×3、C09×3、B04×3、B06×3：全部NOT_RUN |
| C W11完整回归 | S01、S02、S03、S04、S05：全部NOT_RUN |
| D 取消/目标替换 | NOT_RUN |
| E W12 | crossing×3、blocking×3、approach×3、静态S03×1：全部NOT_RUN |

9次已执行物理运行总环境碰撞0、动态障碍接触0、非足接地0、跌倒0、FAULT0；均未arm/发送导航运动目标，不能据此声明未执行导航案例通过。物理INVALID run无；唯一FAIL run为上表09。

## 证据索引

- `verification-plan.yaml`：本轮新矩阵，预注册提交f1fdd05f078b82c989e49fd6e9853251087db661后字节未改。
- `initial-audit.json` / `host-initial-audit.json` / `final-freeze-audit.json`：源码、测试入口、W05/W10契约、上游及历史证据审计。formal-v1 1371、W11R1V原始64、W11R2原始151文件哈希全部匹配；源配置116文件匹配。
- `build*.txt`、`test*.txt`、两份package XML、`evaluator-tests.*`、`static-test-audit.json`：完整构建/164+15测试及13新增测试索引。正向/反向测试stdout/stderr单独保存。
- `phase-A-summary.json` / `summary.json`：全部首次结果，不删除失败。
- `runs/<run ID>/assessment.json`、`closed-runs.json`、`signal-syscalls.json`、`process-and-error-excerpt.txt`：每次进程/参数地图checkpoint manifest/安全事件/真实就绪响应及退出状态。
- `failure-assessment.json`：DDS失败与候选进程、基础设施及安全事件的区分，明确精确退出码证据不足。失败run目录另保存DDS原始日志及combined stdout/stderr。
- `startup-invocation.txt`及其他`*-invocation.txt`：完整调用/纯记录封装，不修改冻结入口或测试逻辑。
- `raw-artifact-index.json`：所有新ignored原始stdout/stderr（既有harness合流记录）、轨迹、结果、进程及工具文件的路径/大小/SHA256。原始运行目录run_data/w11r2-independent-v1/startup/01–09；模拟器run_data/<本轮唯一run ID>/。
- `final-process-audit.json`：宿主无ROS/MuJoCo/RViz/DDS/launch/评测残留，180个登记launch/child/DDS PID全部不存在，42124/42126释放；未停止无关进程。

最终证据提交先于W11记录/看板/交接提交；EVIDENCE_SHA与FINAL_HANDOFF_HEAD由收尾记录及最终回复给出，避免提交自引用。**不满足进入W13 formal-v2的前置条件。** W13 FAILED，W14/W15 TODO。没有执行formal-v2或RTF正式验收，没有启动W14/W15；没有修改源码、launch、参数、场景/地图/模型/控制器、测试入口或行为/判据、冻结契约、旧证据或用户文件，没有push/reset/rebase/clean。

# W13 formal-v2 正式验收

**W13_FORMAL_V2_FAIL**。已启动8/40例，冻结结果完成7例：S01–S06成功，S07有效失败，S08因整轮基础设施中止而受控中断；另外32例NOT_RUN。没有重跑、替换或删除案例。W14/W15保持TODO。

S07在DDS关闭之前已报告 `TERMINAL_CORRECTIONS_EXHAUSTED`、goal工具exit2，正式分类 `FAIL_SAFETY_STOP_UNRECOVERED`，末态真值偏航20.595005483952413°。该有效失败独立否定static10/10；因此使用FAIL，不用基础设施状态掩盖已发生的候选失败。随后DDS退出−11（SIGSEGV），不满足预登记no-crash条件，必须停止整轮。没有开展DDS专项调查或任何修复。

## 提交和冻结

- 开始HEAD：`decd5bf846658976d352d9d0bfb6ca71dbdfdcf2`，分支 `feat/t1-mujoco-navigation`，起始clean。
- REPAIR_SHA：`166b4a15835a68eb6fd7d1e685f0cc68421f7d77`；HANDOFF_SHA：`a4589cbdd65e826d2b9092278e68e21f2322189b`。
- PROTOCOL_SHA：`78c9141a0df3b128450b6954e27a0c570933ab4e`，先于全部物理运行。
- 40例预注册/静态门槛提交：`01927d193847f1ff23c7a403de2c16de2698c7dc`。
- PHYSICAL_TEST_HEAD（全部8例相同）：`ce6309b92ab1c19e87001f24280ab96241d7af1d`。
- EVIDENCE_SHA（原始首次证据独立提交）：`aa97b430d4edf3998bc47840951c820c67d7bed2`。
- 报告与最终交接后续独立提交；最终交接HEAD见git log，避免自引用。
- src/scripts 116文件JSON指纹：`4ffaee2ab9b6e31fdf2eb5920ce7ab27c5e78a8b4cbc616949f6ce616d71c28c`；evaluate行指纹：`8b260c2b963517a2fc7662de25905c1b257752b565d38c3ac35fe5abe1ca646f`。
- REPAIR至物理HEAD/最终审计无src/scripts/架构差异；正式入口与formal-v1收尾版本完全相同。W05/W10、参数、模型/地图/场景/轨迹/种子/阈值均未改，上游原SHA且clean。
- formal-v1原始1371文件、W11R1V64、W11R2原始151、W11R2V原始434索引全匹配；W11R2V仍为 **INDEPENDENT_VERIFICATION_FAIL**。

## 首次结果

| 场景 | 成功/计划 | 启动 | 冻结结果完成 |
|---|---:|---:|---:|
| static | 6/10 | 8 | 7 |
| crossing | 0/10 | 0 | 0 |
| blocking | 0/10 | 0 | 0 |
| sudden | 0/10 | 0 | 0 |

动态三类均未运行，0/10是计划分母及覆盖缺口，不是测得的0%可靠率。INVALID：无。S08已产生物理行为，不满足“行为结果产生前基础设施已失效”的严格INVALID定义；保持 `INTERRUPTED_BY_CAMPAIGN_ABORT`，不记成功、不重跑。

| case | run ID | 结果 | DDS精确码 |
|---|---|---|---:|
| S01 | `20261015T104213Z_w13-formal-v2_S01_s1000_ee3535` | SUCCESS | 0 |
| S02 | `20261015T104312Z_w13-formal-v2_S02_s1001_98e483` | SUCCESS | 0 |
| S03 | `20261015T104413Z_w13-formal-v2_S03_s1002_9ef47d` | SUCCESS | 0 |
| S04 | `20261015T104529Z_w13-formal-v2_S04_s1003_58e684` | SUCCESS | 0 |
| S05 | `20261015T104645Z_w13-formal-v2_S05_s1004_a69103` | SUCCESS | 0 |
| S06 | `20261015T104841Z_w13-formal-v2_S06_s1005_b64989` | SUCCESS | 0 |
| S07 | `20261015T104928Z_w13-formal-v2_S07_s1006_816735` | FAIL_SAFETY_STOP_UNRECOVERED | -11 |
| S08 | `20261015T105049Z_w13-formal-v2_S08_s1007_76e75d` | INTERRUPTED_BY_CAMPAIGN_ABORT | 0 |

8例环境碰撞0、动态障碍接触0、非足接地0、跌倒0、FAULT0。已完成7例用于正式终点指标的最差位置误差 **0.18082195992101752m**，最差偏航误差 **20.595005483952413°**。S08中断时位姿误差另见case-results.json，属于途中观察，不当作到达误差。

## DDS例外判定和采集错误

唯一非零DDS：`20261015T104928Z_w13-formal-v2_S07_s1006_816735`，PID141976，精确returncode **−11 = SIGSEGV**。它在一次预期SIGINT后0.11420075699983817墙钟秒内结束；关闭前仍存活；端口和PID均释放；机器人launch/starter及17子进程exit0；但goal工具exit2，完整的“所有被测子进程正常退出”条件亦不满足；无SIGTERM/SIGKILL。日志只有正常启动和 `Server shut down`，但SIGSEGV本身已证明crash，故 **不适用例外，INFRASTRUCTURE_FAILURE**。其余7例DDS精确码均0。

附加assess.py错误地仅搜索DDS日志的crash/assertion等文字，遗漏returncode−11的信号含义，输出错误的INFRASTRUCTURE_WARNING并启动S08。该错误原assessment和采集器原字节全部保留；发现后立即按已提交no-crash规则中止，不修改例外、不修复采集器、不调查DDS、不重跑。failure-assessment.json明确记录正确裁定，防止把原错误警告当作正式结论。

S08中止链：向本任务编排PID143685发送一次SIGINT→冻结编排器转发worker→原finally关闭launch/goal/authority/DDS。launch父进程和17子进程exit0，DDS0；goal工具在受控中断下exit254并有KeyboardInterrupt，编排shell观察exit130，worker精确整数没有被冻结KeyboardInterrupt路径保存，不猜测。原worker没有生成result.json，这是有解释的中断数据缺口；未补造正式result。S08 action ABORTED/goal FAILED发生于受控关闭期间，不据此归因候选。

机器人/launch非零退出清单：launch父进程0/8非零、launch子进程0/136非零、starter0/8非零；另goal工具S07 exit2（原有效失败）、S08 exit254（受控中止），均完整披露。SIGTERM/SIGKILL：无。最终登记168个launch/child/goal/authority/DDS PID全部消失，宿主相关进程无，42123/42124/42126空闲。

## 严格headless RTF

冻结workload仍是完整40例headless、target_rtf=1.0；未运行GUI或target1.1替代。沿用report_evaluation.py：滚动10墙钟秒内Δsim/Δwall，剔除每条trace最初10墙钟秒以保证完整窗，其余全部样本合并，用未舍入median与1.0比较。原wall_mono/sim_time/rtf_10s都在索引指向的sim_trace.csv。

| 部分数据（8例，含中断） | 原值 |
|---|---:|
| n | 19362 |
| median | 1.0000000631634745 |
| p5 | 0.9237450183137464 |
| p95 | 1.0000071063917308 |
| max | 1.0000616638023765 |
| 低于1.0的样本，全部保留 | 9541 |

该部分样本数值median≥1.0，但 **完整workload严格RTF验收NOT_COMPLETED**；不得用8例代替40例判PASS。performance.json另保留调用原冻结performance()对7例已完成结果的全性能统计及8例逐trace统计；未删除低样本或舍入制造PASS。

## 验证与证据索引

- 四包system colcon build退出0；164包测试+15既有评测器测试，0errors/0failures/0skipped。解释器和环境通过scripts/env.sh，无依赖/环境重装。
- [ACCEPTANCE_EXCEPTION.md](ACCEPTANCE_EXCEPTION.md)：预提交例外；[preregistration.yaml](preregistration.yaml)：40例完整计划/轨迹/顺序/阈值，物理运行后未改。
- [summary.json](summary.json)、[results.csv](results.csv)、[case-results.json](case-results.json)：最终裁定与40行覆盖（32NOT_RUN、1中断）；动态/未完成指标明确不适用。
- [failure-assessment.json](failure-assessment.json)：DDS SIGSEGV正确判定、原误报和S08中断，不改写原始结果。
- [performance.json](performance.json)：冻结方法、逐run样本数/原wall/sim、未舍入统计与原trace SHA。
- [process-summary.json](process-summary.json)：逐run launch/全部17子进程/goal/authority退出码、实际signal审计。
- [runs/](runs/)：每例原result（仅7份）、support-exit、原assessment、request、manifest、CSV、合流console和stdout/stderr日志。S08无result是预期中断缺口。
- [run-inventory.json](run-inventory.json)、[raw-artifact-index.json](raw-artifact-index.json)：307份ignored原始文件的绝对根路径、相对路径、大小、SHA256，原始geometry/完整sim_trace保持ignored。
- [initial-audit.json](initial-audit.json)、[post-abort-audit.json](post-abort-audit.json)、[final-audit.json](final-audit.json)：冻结和宿主/全部168PID核验。初次中文路径引号误报审计亦留存并已解释。
- [raw-byte-preservation.json](raw-byte-preservation.json)：原CSV CRLF和ROS日志尾空格的diff-check告警；保留原字节，不清洗原始证据。
- offline-adjudication-invocation.txt / raw-export-invocation.txt：离线复核/导出逐字调用，不修改被测入口或原数据。

证据索引绝对路径：`/home/haoyang/humanoid_nav_ws/docs/evidence/W13/formal-v2/README.md`。不满足进入W14条件；W14/W15均未启动。无push/reset/rebase/amend、无历史或用户文件改写。

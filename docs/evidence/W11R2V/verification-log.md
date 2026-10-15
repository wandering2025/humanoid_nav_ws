# W11R2V independent verification

2026-10-15T10:15:59.845434+00:00：从头执行新矩阵，起始HEAD a4589cbdd65e826d2b9092278e68e21f2322189b，initial clean。文件冻结审计PASS，116 source/scripts匹配；W05/W10与formal基线无diff，formal-v1 1371、W11R1V raw64、W11R2 raw151全部哈希匹配。尚未构建/启动物理运行。下一命令：宿主审计并提交预注册。

2026-10-15T10:18:10.319570+00:00：静态验证PASS，4包build、164tests/0errors/0failures/0skipped、评测器15/15全部退出0。测试HEAD f1fdd05f078b82c989e49fd6e9853251087db661；新增13关闭测试正反向全过，原XML及子进程stdout/stderr旁路保存。下一命令：从新矩阵STARTUP-01开始，10个新Python/ROS/DDS/launch，先候选失败即停止。

2026-10-15T10:22:39.398911+00:00：阶段A保存首次全部结果，执行9次；1:PASS, 2:PASS, 3:PASS, 4:PASS, 5:PASS, 6:PASS, 7:PASS, 8:PASS, 9:FAIL。逐run ID、manifest、退出及安全事件已旁路导出；原始stdout/stderr/轨迹保留ignored目录。

2026-10-15T10:24:03.811788+00:00：阶段A在09有效失败停止；08前均PASS，09 dds_exit_zero=false，精确DDS returncode未由既有采集器保存。09 starter/18子进程/launch退出0，无traceback/升级/残留，DDS日志正常关闭；根因未证实，不归因starter源码，不改INVALID、不重跑。执行9/38，10及B–E NOT_RUN。最终宿主审计0残留/端口释放；冻结116文件、formal-v1 1371及旧raw64/151全匹配。结论INDEPENDENT_VERIFICATION_FAIL。下一命令：提交本轮失败证据，再更新W11记录/看板/交接，退回修复agent调查DDS/session退出及精确退出码证据缺口。

# W11R2 正常SIGINT关闭修复

**CANDIDATE_READY_FOR_INDEPENDENT_VERIFICATION**；不是最终导航PASS。

起始SHA `c37756d680e5d14654f64954f2bdcf3d79d51d7c`；REPAIR_SHA `166b4a15835a68eb6fd7d1e685f0cc68421f7d77`。
完整根因、修改文件、安全边界、环境与独立验证入口见[REPAIR_HANDOFF.md](REPAIR_HANDOFF.md)。HANDOFF_SHA在证据提交完成后的最终回复给出。

- 精确缺陷：原starter成功输出后，正常SIGINT在node.destroy_node→destroy_client→client.destroy_when_not_in_use中触发未捕获KeyboardInterrupt，退出−2。
- 修复：starter专用私有context/executor；SIGINT只记录请求，在spin边界退出工作；资源清理期间不异步展开析构，幂等cleanup；意外异常保留非零退出。
- 受影响包构建PASS；包测试164项PASS（原151+新增13），额外15项既有评测器单元测试PASS，共179项，0失败/跳过；真实子进程SIGINT exit0，反向异常注入exit1且traceback保留。
- 同一REPAIR源码连续3次真实冷启动/就绪/单次SIGINT PASS，各18/18子进程exit0，无异常/升级/残留；环境碰撞0、跌倒0。未发送导航运动目标。
- W11R1V仍INDEPENDENT_VERIFICATION_FAIL，原64个raw哈希不变；formal-v1 1371哈希不变。W05/W10/导航参数/正式评测入口不变。
- 后续独立验证必须用新候选从头执行完整矩阵。未运行W13 formal-v2，未启动W14/W15。

证据索引：

- `initial-audit.json` / `final-freeze-audit.json`：版本、归属与不可变性。
- `build*.txt` / `test*.txt` / `*-pytest.xml` / `shutdown-tests.*`：命令退出码与自动化测试。
- `test-baseline-audit.json`：原151测试逐项保留、新增13，以及navigate AST不变。
- `subprocess-{normal,unexpected}-{stdout,stderr}.txt`：真实子进程正常退出与反向错误用例原始输出；unexpected里的traceback是有意注入，必须存在。
- `startup-01..03-{assessment,closed-runs,process-audit,signal-syscalls}.json` / `*-excerpt.txt`：逐例真实就绪、进程、信号、参数/地图/checkpoint哈希和退出证据。
- `summary.json` / `final-process-audit.json`：连续3次结果、进程端口终态。
- `startup_once-invocation.txt` / `process_audit-invocation.txt`：逐字采集/审计脚本，仅复用未改动的冻结startup_probe/harness；无修改验证标准。
- `raw-artifact-index.json`：ignored原始运行日志、trace和manifest路径/大小/SHA256。

# W11R1 修复候选证据

状态：CANDIDATE_READY_FOR_INDEPENDENT_VERIFICATION；不是最终PASS。
REPAIR_SHA：`47a383bdeb57f9531485a464dec22d7b2514cdb3`。完整入口：[REPAIR_HANDOFF.md](REPAIR_HANDOFF.md)。

- 四原失败根因：[DIAGNOSIS.md](DIAGNOSIS.md)、root-cause-evidence.json。
- 最终四例单次物理冒烟：smoke-summary.json、C01/C09/B04/B06-result.json、各goal和startup-exit摘录。
- 取消/替换：cancel-replace-result.json；开发期原始命名不兼容FAIL与离线核验均保留。
- 151 tests及四包构建：build-candidate-h13.txt、test-packages-h13.txt、test-result-h13.txt。
- 失败到修复的实验：experiments.json、development-run-index.json；大体积raw和run_data保留ignored。
- 冻结/完整性/进程：freeze-audit.json、formal-v1-baseline-hashes.json、process-audit.json。

没有运行W13 formal-v2、完整独立矩阵或RTF验收，没有启动W14/W15。原formal-v1及历史W11/W12/W13数据没有覆盖。

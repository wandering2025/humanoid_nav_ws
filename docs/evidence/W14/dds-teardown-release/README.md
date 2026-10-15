# W14 DDS 关闭例外：新增发布解释层

本目录不替代 rviz-lifecycle/ 中任何原始结果。publication-assessment.json 的完整02
original_status=FAILED，dds_returncode=-11；publication_status=SUCCESS_WITH_WARNING。
分类为 NAVIGATION_SUCCESS / SAFETY_PASS / DDS_TEARDOWN_WARNING；完整01维持SUCCESS。

用户决策 f687f21922245aa6cace42a9cef204c18f50f9b2；本次没有新GPU查询、图形测试或物理运行。
assess_release_evidence.py 只读原始文件，按被冻结 runner 的 poll→send SIGINT→写信号记录顺序回溯关闭时序；
这不是当时新增的仪器测量。每轮保留结果、日志、Windows检查和清理审计的来源哈希。
原第二轮DDS堆栈缺失，内核定位外部libfastrtps TCP线程；接受例外不代表修复DDS。

verification.json：18项新增分类/证据测试、4项旧finalizer回归、8项GPU规则纯函数回归，共30项PASS。
Python编译/strict shell语法/help/diff-check通过；完整日志ignored且raw-index.json记录hash。
原8组索引506项字节核验全部通过，见original-evidence-integrity.json。
DDS-only halt原字节保存在dds-only-halt-original.json及ignored raw归档，SHA256：
51ff001974ea062a8c1821644869ead973e4246586335653ca9259f42199494f。
只有该锁在用户明确授权后归档；Watch 对任何新halt仍拒绝运行。

final-process-port-audit.json 为本次非图形最终检查：无项目进程，42127/42128可bind。
宿主GPU/Windows事实引用原两轮审计，未刷新GPU；dxg adapter-query ioctl −22/−2未证明无害。
录像 FINAL_VIDEO_PENDING_W15，路径/大小/SHA256 N/A，播放NOT_RUN。W15未开始。

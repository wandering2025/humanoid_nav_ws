# 演示基线接受决策（W14，2026-10-15）

状态：**DEMO_BASELINE_ACCEPTED_WITH_KNOWN_LIMITATIONS**。

用户在本次会话明确授权从首次 W13 formal-v1 交接提交
`b23a92465b317ca781d3eb210051126c5786908e` 创建演示发布分支。
首次实现/证据提交为 `04d74f17a6d56d24d7da5ae8ff0c9a4398608daf`。
本决策覆盖旧计划中“W13 必须通过才能进入 W14”及“保持原分支”的依赖规则。

W13 formal-v1 **仍为 FAILED**：static 10/10、crossing 8/10、blocking 8/10、
sudden approach 10/10，总计 36/40；环境碰撞 0、跌倒 0。
crossing 和 blocking 未达到原每类 9/10；正式 headless RTF 中位数
0.9999998608211204，严格低于 1.0，工程差异很小但仍记 FAIL。
四个失败为 C01 启动定位未完成、C09 action/服务确认超时、B04/B06 末端收敛失败。
原始结果、报告和逐例证据不得覆盖或改写为 PASS。

用户接受该版本作为求职现场演示基线，不再以 40/40、每类 9/10 或所有条件下高度稳定
阻塞演示发布。选择既有成功的固定动态场景，只用于现场复现，不构成新的随机可靠率统计。
这不是安全认证，也不是正式可靠性声明。

W14 只负责演示入口、启动/清理、操作复现、证据和诚实披露；不得修改导航算法、
控制、安全阈值、物理或场景时序来获得通过。两次同命令排练仍须如实验收。
W15 另行分派，负责最终视觉优化、独立复验和演示版本冻结，本次不启动。

起始审计：cwd `/home/haoyang/humanoid_nav_ws`；起始分支
`feat/t1-mujoco-navigation`，HEAD `7b58acd77de722c9b53f4b69271dde862d7f0ff3`，
`git status --porcelain=v1` 为空。新建分支 `release/w13-formal-v1-demo`，没有复用或移动既有分支。
原实验分支仍指向上述 HEAD；后续修复、独立验证和 formal-v2 失败历史全部可达并保留。
未执行 reset、rebase、amend、删除、覆盖历史或 push。
新分支创建后源码与基线提交无差异；formal-v1 summary 为完整 40 行、verdict FAIL，
每行保留实际代码 SHA，整套运行 source/config 指纹
`2b8c5b71825db1b3d0c6afe5a267421f0741d858e01256a3916205a10cbdb5d7`。
收尾评测工具的离线分析变化见原 W13 记录；导航/控制/安全/世界使用首次基线。

本文件独立提交的完整 SHA 即 **DEMO_DECISION_SHA**，在 release manifest 和 W14 记录引用。

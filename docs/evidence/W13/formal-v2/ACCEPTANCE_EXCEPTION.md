# W13 formal-v2 DDS 支撑进程退出验收例外（预登记）

用户于2026-10-15明确授权。本记录须在任何formal-v2物理运行之前提交；该提交为 **PROTOCOL_SHA**，在后续预注册文件写入完整SHA。规则在看到结果后不得修改。只执行W13，串行、无委派，不调查DDS、不执行W11R2完整独立验证矩阵、不启动W14/W15。

## 历史结论保持

W11R2V仍是 **INDEPENDENT_VERIFICATION_FAIL**，不追溯改为PASS或INVALID。失败来自run `20261015T102107Z_w11r2-independent-v1-startup_09_s0_13f20b` 的DDS测试支撑进程一次非零退出检查。该次机器人导航进程、starter、launch父进程及全部18个launch子进程均退出0。DDS日志显示 `Server shut down`；所有登记PID消失，端口释放，无traceback、SIGTERM或SIGKILL。原采集器只保存 `returncode == 0` 布尔值，没有保存精确整数，整数不可恢复，不作猜测。没有证据证明这是导航候选源码缺陷。来源为未改动的[W11R2V结论](../../W11R2V/README.md)及其逐run原始证据。

## 本轮适用规则

从W13 formal-v2开始，DDS支撑进程在正常评测结束后的非零退出码仅记作基础设施警告，不单独判产品失败，**必须同时满足全部条件**：

1. DDS在收到一次预期正常关闭信号（冻结Session.close发送的SIGINT）后结束。
2. DDS在冻结的10墙钟秒期限内结束。
3. 不需要SIGTERM或SIGKILL。
4. 没有DDS crash、assertion、sanitizer或traceback。
5. DDS端口已释放。
6. 没有遗留DDS进程。
7. 机器人launch、starter和所有被测子进程正常退出。
8. 精确DDS整数return code已保存。

以下仍为基础设施失败，不得忽略：运行期间DDS提前死亡；导致ROS通信丢失；无法启动或端口冲突；关闭超时；需要SIGTERM或SIGKILL；关闭后残留进程或端口；精确退出码未保存；或DDS问题影响物理结果有效性。

例外**仅适用于评测结束后的DDS测试支撑退出状态**，不适用于任何机器人、导航、规划、控制、安全、starter或launch进程。本变更是用户授权的验收边界修正，绝不代表W11R2V通过。导航成功率、安全、定位/终点误差、40例场景/顺序/种子/轨迹/超时、严格headless RTF标准完全不变。正常结束后的受控DDS非零退出本身不构成INVALID。

精确退出码采集仅在冻结关闭调用返回后读取Popen.returncode并写入附加证据，不改变原Session.close、信号顺序、超时、场景或冻结结果。原stdout/stderr保持冻结harness合流方式，合流事实明确记录，不声称分离原本未分离的流。

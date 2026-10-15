# 附加退出码采集静态审查

正式 scripts/evaluate.py 及 scripts/evaluation/* 与formal-v1收尾版本逐字相同。capture.py仅给worker增加Python bootstrap，sys.argv恢复为正式入口原值；正式run()仍创建每例request/manifest、调用正式worker和全部冻结判据。

worker只包装Session.close：原方法恰好调用一次，原rclpy.shutdown→DDS一次SIGINT→wait(timeout=10)→log.close保持；不添加sleep、信号、重试或异常吞掉。返回值与异常原样传播。在finally读取Popen.returncode，保存整数（包括非零）或明确null，附加信号审计。launch、goal、authority的Popen退出值也只读保存，不修改原result.json。AST隔离测试确认0/7/-2原样保存、原返回值/单次调用顺序/TimeoutError传播保持。

DDS关闭前poll用于记录其是否已提前死亡；没有修改候选、参数、输入、clock、评测结果或原signal调用。stderr/stdout继续由正式harness合流；原本合流的流不能追溯拆开。每例评估support-exit与原日志，例外只按已提交ACCEPTANCE_EXCEPTION.md判断，原正式SUCCESS不自动代表总验收通过。

冻结RTF只使用正式40例headless target1.0的完整10wall秒窗；调用原report_evaluation.performance函数读取原trace，未舍入median>=1.0。无专用target1.1/GUI替代，无源码性能优化。

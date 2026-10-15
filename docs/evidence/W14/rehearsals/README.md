# W14 默认GUI正式排练：完整失败历史

最终W14_BLOCKED；每个目录保留原result和独立final-result、preflight SHA/dirty和完整raw索引。
原始JSON不覆盖；不把到达或action成功单独记为整轮通过。所有命令均为 `./scripts/run_demo.sh`，固定U01/4000。
每批在运行前记录到W14记录/交接；同一批源码干净且相同，新PTY不继承ROS/venv。

- batch1：SHA23217d6b79332171c2f4ccb91cdf85218559352d，01 SUCCESS；02启动观察器无clock FAIL。未发目标，物理trace13.42s。
- batch2：SHA2a211be8b6f4f94bf5cb4db40e20e9848ab30323，01初始零clock被错误判断为停滞 FAIL；02 NOT_RUN。未发目标。
- batch3：SHA17aae04e368614f97eb5629a8d855a6c732ec719，01到达/dynamic PASS但RViz退出堆损坏−6，整体FAILED；02 NOT_RUN_SAFETY_HALT。

最后两轮预登记中仅01执行（run `20261015T132133Z_demo_U01_89a56cbe-0723-4ca9-b438-1021c94912e9`）；不能把不同源码的batch1/01和batch3/01合并成同版本两次通过。
此前原黑屏事故、CPU辅助0.42s FAIL、站立端口失败、CPU回放DDS−6另在上级证据目录，均保留。

安全停车和无接触/跌倒通过；最终端口释放，最后全部本项目进程消失。RViz−6不能写成0，具体原因未定位。
GPU采样未触资源停止线；最新Windows查询无新匹配Display/LiveKernelEvent/异常电源事件。两者都不是故障隔离保证。
最后实现/封锁SHA 457dc8f01ffac88582e6017c631dd4a6f2d9f3a6；仅非图形检查，不再GPU重试；录像MISSING，W15未启动。

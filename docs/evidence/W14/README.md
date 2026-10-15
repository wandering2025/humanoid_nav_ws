# W14：BLOCKED / 宿主图形事故

事故run：20261015T114444Z_demo_U01_6c4ff40d-5119-4308-8252-a71715a2a1b7；
clean HEAD b1ab66fecace5aff3acf4ed6fbefa4cfb85ac3af，默认命令，只运行一次，未发送导航目标。

- incident-summary.json：排练/进程退出/指标缺失/重启后核查。
- launch-excerpts.txt：带原行号的D3D12、插件残留、关闭升级/退出错误。
- kernel-excerpts.txt：前一启动11:44:54–59 UTC图形层错误，含libgallium线程段错误。
- windows-event-excerpts.json：Intel DX12 pagefault、NVIDIA TDR、Kernel-Power41/EventLog6008。
- raw-artifact-index.json：原始日志、partial trace/manifest与只读采集结果的路径/大小/SHA256。
  Windows完整日志可能含本机账户信息，保留ignored，不提交；这里只提交相关事故摘录。
- raw/incident-20261015/：只读采集、重启后host /proc audit、静态检查；ignored。

初始4包colcon build成功（3.25s），日志run_data/demo/build-W14.log；6项静态/隔离测试PASS，
bash -n/py_compile/git diff --check PASS。没有因此宣称运行、图形安全或清理PASS。
原生Windows关闭终端/重新打开界面未由agent操作；实际是独立Ubuntu PTY的干净环境启动，
不能写成已完整验证Windows用户手工开关终端流程。

未运行第二次排练、录像、修复后的ROS/GUI动态验证。恢复需要用户另行审查授权，不允许解锁重试。

> 后续状态更新：用户再次要求在电脑安全前提下继续W14。默认GUI仍封锁；
> 已增加Landlock GPU设备拒绝规则，仅开放CPU预检/--headless。
> 唯一CPU辅助run到达且正常停车/回收，但动态保护区静止持续0.42s<冻结0.5s，仍FAIL；不重跑物理。
> 见[cpu-recovery-plan.md](cpu-recovery-plan.md)及[独立证据](../evidence/W14/cpu-recovery/README.md)。
> 以下为原事故调查记录，事故结果和缺失退出码保持不变。

# W14 宿主黑屏事故与安全审查

状态：**W14_BLOCKED**。2026-10-15；事故约 13:44–13:48 Europe/Berlin（11:44–11:48 UTC）。
用户报告 MuJoCo 标志闪现后整机黑屏、鼠标不可见，只能按电源强制重启。
这是宿主级严重故障，不能标为普通 GUI 退出问题或 W14 完成。

## 已证实的证据

- 唯一默认排练：`20261015T114444Z_demo_U01_6c4ff40d-5119-4308-8252-a71715a2a1b7`。
  命令 `./scripts/run_demo.sh`，被测干净 HEAD `b1ab66fecace5aff3acf4ed6fbefa4cfb85ac3af`。
- 新 Ubuntu PTY 没继承 ROS/venv 环境；保留已有 PYTHONPATH，不通过清空它绕过集成。
  首次干净终端预检发现软件 GL，入口新增仅本进程 `GALLIUM_DRIVER=d3d12` 后再预检。
  没有设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME`；实际 GL 选中 **Intel Arc Pro**。
  先前继承会话中的预检是 **NVIDIA RTX 4050**；CUDA 可用并不能证明 OpenGL 使用 NVIDIA。
- launch.log 第139、143行：MuJoCo 和 RViz 均报 `D3D12: Removing Device.`。
- 前一启动内核 11:44:55–56 UTC：`dxgkio_wait_sync_object_cpu` 返回 -19，
  `dxgkio_create_allocation` 返回 -75，随后 `python:gdrv0` 在
  `libgallium-25.2.8-0ubuntu0.24.04.4.so` 段错误。该线程尚未通过 dump 映射到具体应用，
  不仅凭线程名就断言一定属于本次 MuJoCo。
- Windows WER 11:45:24 UTC：`LiveKernelEvent 141`，bucket 为
  `LKD_0x141_Tdr:C_IMAGE_igdkmdn64.sys_MTL_DX12_PAGEFAULT`。
  随后 11:46:59、11:47:23 记录
  `LKD_0x141_TdrFlag:UNKNOWN_ERROR_Tdr:6_IMAGE_nvlddmkm.sys_Ada`。
  **Intel 与 NVIDIA 两条驱动链均出现超时记录**，不能认为切换 NVIDIA 就一定安全。
- 重启后有 Kernel-Power 41、EventLog 6008，符合用户报告的非正常关机。
  Windows 原始事件和完整 Linux 原始日志保持 ignored，轻量摘录与 SHA 索引提交。
- 持久化 trace 275行，最后 sim=5.50s（终端曾观察5.8s）；所有策略输入为0，
  仅 STARTING/READY；门为 DISARMED，尚未发导航目标、未完成初始位姿/就绪。
  没有完整 result.json，manifest exit_code=null；不能将缺失的全程碰撞/跌倒/净距填为0。
- ROS 关闭记录中，initial_pose/starter 各 exit1，localization manager exit−9；
  MuJoCo/RViz 收到 SIGKILL 但没有可恢复的最终退出整数，必须记 unknown。
  Ctrl+C 后 rclpy 默认信号处理使 context 先失效，finally 内 graph 读取再抛异常，
  导致本次编排清理没有正常完成；这也是 W14 自身缺陷。
- 另有旧实验 install 残留的 plugins.xml 索引错误，与图形错误同时存在；
  它能影响 ROS 启动，但不能据此解释 Windows 双显卡 TDR。未删除/移动 build/install。

## 原因判断与不确定性

目前最有依据的判断：此次双窗口启动触发/暴露了 **WSLg/Mesa D3D12 → Windows 显示驱动**
链路异常，Windows 显示引擎超时与恢复异常和黑屏时间吻合。首个明确 bucket 指向 Intel DX12
pagefault，随后 NVIDIA 也报超时。还未分析 Windows WATCHDOG dump，不能确定最初缺陷
究竟在应用调用、Mesa/WSLg、某个驱动或硬件，更不能断言显卡损坏、显存耗尽或某版本必然有 bug。
这发生在导航目标发送之前，没有证据指向导航算法、机器人碰撞或安全阈值。

微软将 0x141 定义为显示引擎未及时响应的 live dump 标识，**不是已证实的蓝屏代码**。
参见 [0x141 官方说明](https://learn.microsoft.com/en-us/windows-hardware/drivers/debugger/bug-check-0x141---video-engine-timeout-detected)、
[TDR 官方机制](https://learn.microsoft.com/en-us/windows-hardware/drivers/display/timeout-detection-and-recovery)。
WSLg 明确区分 Mesa 的 GPU adapter 选择，见
[官方 GPU selection](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg)。
这些资料说明机制；本机根因判断来自上述实际日志，不以网页替代本机证据。

## 打包审查与已采取的防护

入口预检只排除软件渲染、检查 CUDA 可用，未核验实际 GL adapter；我在这一点审查不足。
仅启用 d3d12 就继续双窗口启动，未建立经验证的 GPU 选择和图形故障隔离门。
宿主驱动挂起也超出 ROS 停车、shell trap、PID 清理的保护能力。

- `run_demo.sh` 当前在 source 环境和任何子进程之前直接拒绝运行，exit3；包括默认、
  --preflight、--headless、--no-rviz、--scenario。仅 --help 可用。内部 Python main 同样锁定。
- 已移除自动强设 GALLIUM_DRIVER 的代码。不是静默回退软件渲染，也没有改为偷偷使用另一显卡。
- 保留启动、失败和信号升级记录。补充 rclpy 不接管编排信号、graph 失败不跳过 cleanup 的代码，
  只通过静态检查，**未进行 ROS/GUI 动态复验**，不能宣称该修复已验证。
- 用户报告后没有再次运行 MuJoCo、RViz、glxinfo、nvidia-smi、CUDA、ROS或 headless 演示。
  仅查询已有日志、文件和 /proc；没有改驱动、BIOS、TDR 注册表、依赖、安全/导航参数。
- 重启后宿主 /proc 只读审计：本项目 ROS/MuJoCo/RViz/DDS进程无；TCP42123/42124/42126/42127无监听。
  这是**重启后观察**，不能冒充脚本安全清理通过。

## 解除阻塞前的后续工作（未执行）

1. 先离线分析 Windows 的 `C:\Windows\LiveKernelReports\WATCHDOG\WATCHDOG-20261015-1344.dmp`、
   `...-1345.dmp`、`...-1346.dmp`；核对相关显示驱动版本和 WSLg/Mesa 版本。
   不改 TdrDelay/TdrLevel，不自动升级、重装或降级驱动。
2. 形成用户可审查的恢复方案，解释双显卡故障及应用/驱动边界。
   任何重新触及 GPU、图形或仿真的实验须另获明确授权；不能因为脚本写好了就解锁重跑。
3. 经授权后才分阶段验证单个渲染器、停止/回收及 GPU 选择，再考虑双窗口。
   ROS 冷启动、Ctrl+C/TERM/异常清理也需独立验真；旧 overlay 残留先保留并核对后再重建。
4. 最后仍需同一发布版本、同一默认命令、同一 seed 两次完整到达及零碰撞/跌倒、无残留，
   完成录像和文档审查，才能重新评估 W14_DEMO_READY。当前第二次排练 NOT_RUN。

证据入口：[事故摘要](../evidence/W14/incident-summary.json)、[Windows事件摘录](../evidence/W14/windows-event-excerpts.json)、
[Linux摘录](../evidence/W14/kernel-excerpts.txt)、[原始哈希索引](../evidence/W14/raw-artifact-index.json)。

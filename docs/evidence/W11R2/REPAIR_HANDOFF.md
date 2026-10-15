# W11R2 shutdown repair handoff

状态：**CANDIDATE_READY_FOR_INDEPENDENT_VERIFICATION**。仅限正常SIGINT关闭修复候选，不是完整导航独立验证PASS。
W11R1V保持 **INDEPENDENT_VERIFICATION_FAIL**；W13 FAILED；W14/W15 TODO。当前模型串行执行，无委派。

## 版本和归属

- 起始SHA：`c37756d680e5d14654f64954f2bdcf3d79d51d7c`。
- **REPAIR_SHA：`166b4a15835a68eb6fd7d1e685f0cc68421f7d77`**。
- **HANDOFF_SHA：留空**，本文件所属证据提交完成后在最终回复给出；避免提交自引用。
- 分支：`feat/t1-mujoco-navigation`；cwd：`/home/haoyang/humanoid_nav_ws`。
- 开始工作区clean，没有用户dirty文件。`docs/start_commands.md`未修改、暂存或提交，SHA256仍为`6973bb1c423297cebdf2c1a97391487023d3b4af955e3a51d4fc735b2fba7027`。
- 构建/测试使用提交前与REPAIR完全相同的源码；三次完整冷启动manifest的code_sha均为REPAIR_SHA。运行时dirty仅W11R2证据、records/W11、看板、交接，无源码/参数dirty，逐例原始清单保留。交接提交后要求clean。
- `src`+`scripts`逐文件哈希见[冻结审计](final-freeze-audit.json)，其JSON排序编码SHA256为`4ffaee2ab9b6e31fdf2eb5920ce7ab27c5e78a8b4cbc616949f6ce616d71c28c`；REPAIR后源码未改变。

## 精确根因

原失败run：`20261015T093701Z_w11r1-independent-v1-startup_01_s0`。原日志[完整失败证据](../W11R1V/README.md)及[行号摘录](../W11R1V/startup-01-excerpt.txt)均未改写。

原combined stdout/stderr第199行完成全部就绪检查并输出成功，第200/202行冻结startup_probe正常关闭launch并转发一次SIGINT；第219–234行调用链：

```text
console entrypoint → starter_main → starter finally
navigation_tools.py:55 node.destroy_node()
rclpy/node.py:1960 self.destroy_client(self._clients[0])
rclpy/node.py:1870 client.destroy()
rclpy/client.py:232 self.__client.destroy_when_not_in_use()
KeyboardInterrupt
```

第300行starter PID99620退出−2。其余17个子进程退出0，launch/DDS/wrapper退出0，无升级或遗留，不能抵消starter异常。

精确发生层是**节点析构内的service client销毁**。不是spin/executor报错；尚未执行到原finally后面的`rclpy.shutdown()`，也没有重复清理的证据。`except Exception`只围住启动工作，既不捕获继承BaseException的KeyboardInterrupt，也不覆盖finally中的destroy_node。成功JSON发布后测试立即关闭进程，暴露了这一合法时间窗口。

本机Jazzy源码另确认：`Node.destroy_client`先从`_clients`移除对象，再调用`client.destroy()`。因此只捕获KeyboardInterrupt后返回0会中断清理；简单重试destroy_node也可能跳过已移除的client。修复须避免信号异步展开析构调用栈，不是忽略−2或吞掉异常。

## 实现范围与清理顺序

源码/测试提交只有以下5个文件：

| 文件（均在src/t1_bringup/） | 目的 |
| --- | --- |
| t1_bringup/starter_runtime.py | 新增starter专用资源所有者；私有Context、SingleThreadedExecutor、协作SIGINT、幂等cleanup |
| t1_bringup/navigation_tools.py | 仅starter生命周期接入；明确正常中断分类；意外异常保留非零退出 |
| t1_bringup/navigation_readiness.py | 可选spin_once回调，让starter使用自己的executor；默认navigate路径不变 |
| test/test_starter_shutdown.py | 10项单元测试：顺序/幂等/中断边界/关闭context/超时/异常分类 |
| test/test_starter_shutdown_ros.py | 3项真实ROS测试：ready后client销毁期间SIGINT、意外析构错误、context已关闭后重复cleanup |

SIGINT处理器只置interrupted，不执行ROS操作、不抛异常。工作在spin轮询边界抛KeyboardInterrupt并进入清理；每次spin最多等待20ms，已有服务发现/响应等待仍遵循原上限，未增加。清理期间该处理器一直有效，因此中断到达client销毁时只记录请求，当前析构完整执行。没有新增sleep或重试。

清理顺序：cancel timers → executor.remove_node → executor.shutdown（2墙钟秒有界等待，超时抛错）→ node.destroy_node（销毁订阅/clients/services/timers等）→ context.try_shutdown。成功后closed锁存；重复cleanup不再销毁资源。context先被外部关闭也可以cleanup；try_shutdown自身原子检查，不使用原来ok()/shutdown()分离检查，不重复调用rclpy.shutdown。独立context避免默认global executor的隐式关闭回调；原信号处理器在资源释放后恢复。

`KeyboardInterrupt`是正常中断；`ExternalShutdownException`只有context确实关闭才按正常关闭处理，活动context下继续抛出。启动协议失败仍退出1。其它工作异常、executor超时、析构异常都传播，绝不宽泛捕获后返回成功。真实subprocess测试在同一销毁位置注入RuntimeError，观察到**退出1且原始traceback保留**；见[subprocess-unexpected-stderr.txt](subprocess-unexpected-stderr.txt)。它是有意的反向测试，不是三次正常冒烟异常。

检查了navigation_adapter、command_gate、initial_pose、navigate工具和simulation的关闭模式。部分也有finally/destroy_node写法，但本失败堆栈只指向starter，原17个其它子进程正常退出；本轮三次也均正常。没有证据要求扩大修改，因此其源码不动；这不构成其它信号时序的全面证明。

## 验证结果

环境通过`scripts/env.sh`加载，system colcon，真实仿真显式venv。命令与结果：

```bash
/usr/bin/colcon build --symlink-install --base-paths src --packages-select t1_bringup
/usr/bin/python3 -m pytest src/t1_bringup/test/test_starter_shutdown.py src/t1_bringup/test/test_starter_shutdown_ros.py -v
/usr/bin/colcon test --base-paths src
/usr/bin/colcon test-result --all --verbose
```

受影响包t1_bringup构建退出0；新增关闭测试13/13；全部4包测试命令退出0；**164 tests / 0 errors / 0 failures / 0 skipped**（bringup80、mujoco84）。另运行colcon之外的既有评测器单元测试15/15通过，共179项不同测试通过（新增13的单独运行不重复计数）。首次额外测试误用系统Python，1项因缺mujoco失败；原14PASS/1FAIL输出完整保留于evaluator-tests-wrong-interpreter.*，改用冻结venv解释器后15PASS，无源码/依赖修改。基线151项按classname/name逐项比对全部保留，新增13；详见[test-baseline-audit.json](test-baseline-audit.json)、[test-result.txt](test-result.txt)和两份JUnit XML。

真实subprocess测试创建真实ROS lifecycle/service/action及clock/scan/TF/AMCL就绪消息；实际starter完成8个active响应、FollowPath ACK/预期103和两个costmap响应。测试在`Client.destroy`内、Node已移除client后设置同步点；父进程等ready及同步点后**只发一次SIGINT**，释放同步点，要求5秒内exit0、stdout/stderr无traceback且进程组不存在。反向用例同位置注入RuntimeError要求exit1。ROS夹具不是物理证据；以下3次使用真实完整launch，无同步点或mock。

| 连续冷启动run ID | 就绪 wall/sim 秒 | starter退出 | launch子进程退出0 | 结果 |
| --- | --- | --- | --- | --- |
| 20261015T095929Z_w11r2-shutdown-v1-startup_01_s0 | 14.6731 / 10.00 | 0 | 18/18 | PASS |
| 20261015T100053Z_w11r2-shutdown-v1-startup_02_s0 | 14.3486 / 11.68 | 0 | 18/18 | PASS |
| 20261015T100150Z_w11r2-shutdown-v1-startup_03_s0 | 14.3202 / 11.58 | 0 | 18/18 | PASS |

每次全新Python/ROS/DDS/launch，RViz启用，static_world、start/initial(0,0,0)、seed0。使用**未改动**的W11 startup_probe(1)，仅绑定现代harness与新输出目录，与独立验证失败调用方式相同。ready后立即关闭，无添加延迟。真实信号syscall审计只记录launch父进程和DDS各一次SIGINT，launch日志确认starter恰好收到一次SIGINT。三次共54个子进程正常退出，wrapper/launch/DDS也全退出0；无traceback、SIGTERM/SIGKILL升级，无遗留进程或端口。

三个运行环境/障碍/非足触地碰撞各0、跌倒0，策略命令全零；未arm、未发NavigateToPose运动目标。FollowPath空path返回103是既有无运动就绪探测。位置/偏航到达误差与终点稳定时间本轮不适用，未声称完成导航矩阵。每例参数/场景/地图/checkpoint哈希、启动trace、process manifest和退出清理见`startup-0x-closed-runs.json`、`startup-0x-assessment.json`及[summary.json](summary.json)。resolved参数哈希因run_id不同而不同，其它控制参数未改。

## 冻结及原始证据

W05 command map/速度包络/禁倒退横移、W10 1.65m停止区/1.95m减速区/链路/锁存/时限均未改。导航行为、控制器、costmap、动态障碍、案例、seed、地图、BT、goal tolerance及120sim秒任务限制未改；新增代码无cmd_vel发布者。`navigate`及其入口AST与起始提交完全一致。W13正式评测入口/scripts也未改。

两个外部upstream保持原SHA且clean。W11R1V追踪文件及64个原始索引文件全部原字节；formal-v1的1371个哈希全同。W11/W12/W13历史及docs/results无改动。新原始运行数据在ignored `run_data/w11r2-shutdown-v1/`与上述run ID目录；[raw-artifact-index.json](raw-artifact-index.json)记录151个文件哈希。执行时的两个采集脚本保存在raw的tools目录，逐字文本版本为`startup_once-invocation.txt`、`process_audit-invocation.txt`，这是调用/审计记录，不修改冻结评测器。

## 当前环境、进程及复现

最终只读审计时间2026-10-15T10:03:47Z：项目ROS/MuJoCo/RViz/DDS/评测进程0；三次共60个已观察launch/子进程/DDS PID全部消失；42124/42126无监听。没有关闭任何无关进程。详见[final-process-audit.json](final-process-audit.json)。

从Windows进入：

```powershell
wsl -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws
```

Bash环境：

```bash
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42124 ROS_DOMAIN_ID=34
source scripts/env.sh
```

额外15项评测器单元测试命令（venv内仅追加系统pytest，不安装包或改PYTHONPATH）：

```bash
"$T1_VENV/bin/python" - <<'PYTEST'
import sys
sys.path.append('/usr/lib/python3/dist-packages')
import pytest
raise SystemExit(pytest.main(['scripts/evaluation/tests', '-v']))
PYTEST
```

受控测试夹具使用独立DDS42126/domain134。冷启动用新目录与ID（不要复用本轮01–03）：

```bash
export T1_EVAL_OUT="$PWD/run_data/<new-campaign>/startup/01"
export ROS_LOG_DIR="$T1_EVAL_OUT/raw/ros-log"
export T1_EVAL_RUN_ID="<new-unique-run-id>"
mkdir -p "$T1_EVAL_OUT"
"$T1_VENV/bin/python" < docs/evidence/W11R2/startup_once-invocation.txt
```

仅复现入口；本轮已完成，不自动再执行。

## 下一部分必须从头独立验证

**不能从旧矩阵STARTUP-02或阶段C续跑，也不能把本轮3次冒烟计入独立样本。** 新候选、新campaign、新证据目录，先只读核对SHA/dirty/进程/原证据，再冻结独立验证计划。按原[独立矩阵](../W11R1V/verification-plan.yaml)从阶段A开始：全构建与全部测试（现在至少164）→10次冷启动→C01/C09/B04/B06各3次→完整W11 S01–S05→取消/stop锁存/rearm/替换→W12 crossing/blocking/approach各3次及S03→最终冻结和进程审计。保持原标准、失败分类与首次候选失败停止规则。

独立验证不得修改src、scripts、launch、参数、地图/场景/案例/种子/事件/BT/checkpoint、测试源码或既有证据，包括W11R1、W11R1V与W11R2；只在新目录追加验证产物/状态。若发现候选缺陷则保留FAIL退回修复，不现场调参或放宽退出标准。再次验证代码差异：REPAIR_SHA至HANDOFF_SHA应仅为W11R2证据和3份计划记录。

第二部分第一条命令必须是只读：

```bash
cd /home/haoyang/humanoid_nav_ws && git status --short --branch && git log -8 --oneline --decorate && git rev-parse HEAD
```

仍未证明：新候选完整导航可靠率、动态避障矩阵、全部中断时序/信号风暴。三次冷启动和定点故障注入证明本缺陷关闭路径，不能替代独立导航验证。W11R1V失败历史不变。本轮没有重跑C01/C09/B04/B06，没有运行W13 formal-v2/RTF验收，没有启动W14/W15。

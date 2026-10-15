# W11R1 第一部分修复交接

状态：**CANDIDATE_READY_FOR_INDEPENDENT_VERIFICATION**。更新时间：2026-10-15T09:09:02.879748+00:00。
这是冻结候选，**不是最终PASS**。W13仍FAILED，W14/W15仍TODO；原W11 DONE历史保留。

## 版本与工作区

- REPAIR_SHA：`47a383bdeb57f9531485a464dec22d7b2514cdb3`。
- HANDOFF_SHA：留空。本文所在提交不能自引用；提交完成后由最终回复给出完整SHA。
- 起始SHA：`b23a92465b317ca781d3eb210051126c5786908e`；分支`feat/t1-mujoco-navigation`；cwd`/home/haoyang/humanoid_nav_ws`。
- 最终五次物理运行均使用REPAIR_SHA；之后只提交本目录证据及W11记录/看板/交接，不修改任何源码或测试入口。
- `src/`+`scripts/`完整指纹：`5ea0ccb65508bd4c70030759fb0d78aa7b382c49e2e303ca7858ea706117fe58`。
- 开始dirty仅用户`docs/start_commands.md`，本任务从未编辑、暂存、恢复或提交。其SHA256始终为`6973bb1c423297cebdf2c1a97391487023d3b4af955e3a51d4fc735b2fba7027`。
- 期间观察到外部提交`e50accf1cb7498a1cd2fe4fc27c7f81152d44aa6`仅提交上述文档同一字节内容；未回退它。不能从通用author推断操作者。本任务未push，未改外部upstream。
- 候选冻结时工作区clean；最终交接提交后的状态由最终回复和`git status`确认。raw/run_data/build/install/log保持ignored。

## 四个原始失败的根因

| 案例 | 原始证据与根因 | 对应修复 |
| --- | --- | --- |
| C01 | map_server实际配置完成却明确记录change_state响应发送超时；manager无界等待，AMCL未激活，initialpose失败、map→odom缺失，未发目标。底层DDS丢响应机制尚未证明。 | RPC有界、状态读回避免重复转换，事件就绪门，失败结束。 |
| C09 | 各节点及costmap已经active；20ms BT确认预算先超时，约0.27墙钟秒后FollowPath才被执行；clear请求已到服务端但响应等待超时。ABORTED且error_code0，旧适配器可能接受孤立旧命令。 | 真正ACK/结果与服务响应探测、500ms有界BT预算、严格状态判断和命令代次清理。 |
| B04 | 预目标在0.35m位置抢占时朝向约95°，未实现沿终点朝向进近；过冲进入东墙2.05m规划膨胀区，静态墙不能被clear，最终208。 | 联合进近状态边界、默认前进朝向格点路径、终点独立核验、有界过冲处理。 |
| B06 | 预目标抢占时朝向约92°；过冲后小速度/转向请求无法驱动步态，无倒退回收；终目标始终未SUCCEEDED，所以旧的成功后积分修正从未进入，最终120s超时。 | 可执行前进/原地路径、小转向意图保留、精确终点朝向、有界三次稳定后修正。 |

原始run ID、日志/result哈希、TF/lifecycle/action、轨迹入口和具体行号见[DIAGNOSIS.md](DIAGNOSIS.md)、[root-cause-evidence.json](root-cause-evidence.json)。formal-v1全部1371文件重新核对无变化；索引及审计见[freeze-audit.json](freeze-audit.json)。

## 修改文件与行为

| 文件 | 目的 |
| --- | --- |
| `src/t1_bringup/package.xml` | 真实bond测试依赖。 |
| `src/t1_bringup/t1_bringup/navigation_adapter.py` | 取消/中止/抢占清缓存，只接受当前goal与FollowPath代次；保留小转向意图，仍唯一导航源。 |
| `src/t1_bringup/t1_bringup/navigation_launch.py` | 有界导航manager；starter失败明确结束launch。 |
| `src/t1_bringup/t1_bringup/navigation_protocol.py` | 有限等待、严格action状态、稳定窗、共享三次纠偏预算、过冲边界与命令代次。 |
| `src/t1_bringup/t1_bringup/navigation_readiness.py` | 传感器/定位/生命周期及真实FollowPath ACK、结果与clear服务响应就绪门。 |
| `src/t1_bringup/t1_bringup/navigation_tools.py` | 120s内端点就绪→发目标→稳定核验；默认完整朝向路径，显式分段需完成并联合核验；失败stop+cancel。 |
| `src/t1_bringup/t1_bringup/slam.py` | 仅navigation模式使用有界定位manager。 |
| `src/t1_bringup/test/test_bounded_manager_ros.py` | 真实ROS服务丢ACK不重放、缺服务有限失败、实际bond形成后正常SIGINT。 |
| `src/t1_bringup/test/test_controller_feasible_ros.py` | 真实controller action验证近终点请求在W05步行带、大朝向误差先转不前进、取消返回5。 |
| `src/t1_bringup/test/test_heading_planner_ros.py` | 真实规划服务检验常规/近障碍起点、绕岛前进几何和精确非格点终点yaw。 |
| `src/t1_bringup/test/test_navigation_adapter.py` | 真实停滞输入及阈值边界；密集输入网格始终落在冻结W05可行输出集合。 |
| `src/t1_bringup/test/test_navigation_protocol.py` | 启动、RPC、ABORTED0、回弹、过冲、预算和命令代次故障注入。 |
| `src/t1_navigation/CMakeLists.txt` | 构建manager和planner插件。 |
| `src/t1_navigation/config/navigation.yaml` | 有限500ms BT确认预算、前进格点规划、可执行速度带的RPP路径跟随、W11 raw转向意图阈值.05；W05实际输出带不变。 |
| `src/t1_navigation/package.xml` | 声明已安装ROS库/插件依赖，无环境升级。 |
| `src/t1_navigation/plugins.xml` | 注册精确终点规划插件。 |
| `src/t1_navigation/src/bounded_lifecycle_manager.cpp` | 有限RPC、丢ACK后读回状态、保留bond契约、失败停止及pre-shutdown清理。 |
| `src/t1_navigation/src/exact_goal_lattice.cpp` | 复用已安装格点搜索，只恢复精确终点orientation，避免纠偏目标被量化。 |

另有`docs/architecture/navigation.md`、追加ADR-019至027、W11R1的`smoke.py`/`cancel_smoke.py`、诊断/实验和过程检查点。ADR-016等历史决策未重写。局部控制最终采用已安装RegulatedPurePursuit（.33–.40m/s前进、航向差>.60rad先转、原地目标.60rad/s、禁倒退、collision detection开启）；MPPI/RotationShim不再启用。所有实现均在REPAIR_SHA及其祖先中，后续交接差异必须为证据/记录。

新流程：READY、clock推进、fresh scan、AMCL确认/TF、active状态、实际RPC全部通过后才arm/发目标。节点发现不能代替可用性。FollowPath空path探测只预期INVALID_PATH103且无导航命令代次；不会驱动物理机器人。原3s/8s创建延迟仅安排负载，正确性来自事件协议。

默认直接给全局规划器最终位置和朝向。已安装diff格点允许前进曲线及原地转向，禁止reverse expansion；只恢复精确末点orientation，全部路径平移样本和圆形footprint碰撞检查保持。显式旧分段选项需action完成并在位置/朝向联合稳定核验后才进入下一阶段。终点纠偏仍共享最多3次、120s总预算；失败stop锁存并取消。Nav2 SUCCEEDED不替代稳定后的定位/独立真值核验，ABORTED0永远不是成功。

## 没有降低安全标准

W05 command_map/输出速度包络、前进物理死区、倒退/横移禁用均逐字节不变。W11 raw转向意图阈值由.20改.05，不是修改物理死区：实际原地输出仍为.60rad/s目标且落在冻结[.56,.85]带。所有整形都在W10之前。

W10 stop1.65m/slowdown1.95m、安全顺序gate→smoother→Collision Monitor→bridge及安全文件不变。唯一`/cmd_vel`发布者仍Collision Monitor；唯一`/t1/cmd_navigation`发布者仍adapter。没有启用spin、backup、横移、倒退恢复，没有真值控制、base-pose移动或硬编码速度轨迹。

位置≤.30m、偏航≤20°、.5s窗口线/角速度均<.05且持续2s、120s仿真上限不变。内部GoalChecker/工具仍使用更紧的.20m/.20rad，未放宽。场景/地图/轨迹/种子/案例、BT和W13评测器均未改。完整冻结路径与哈希核对见freeze-audit。

## 自动化与构建

通过命令（环境见下）：

```bash
/usr/bin/colcon build --symlink-install --base-paths src
/usr/bin/colcon test --base-paths src
/usr/bin/colcon test-result --all --verbose
```

4包构建成功；151 tests，0 errors，0 failures，0 skipped。保存`build-candidate-h13.txt`、`test-packages-h13.txt`、`test-result-h13.txt`。45项协议/适配器定向测试通过；真实ROS规划测试4例、controller行为测试2例通过。精确yaw回归在修复前2FAIL/2PASS，修复后全部通过，前置失败日志保留。没有为了掩盖错误升级依赖。

覆盖：lifecycle未ready禁止通过、initialpose/TF缺失明确失败、被发现但不ACK/拒绝的action、发现但不响应的costmap、FollowPath ACK超时、ABORTED0、终点过冲边界、回弹重启稳定保持、三次耗尽、取消清缓存及W10锁存、抢占等待新FollowPath及新命令、单一合法命令链。纯协议故障注入、真实ROS服务测试和物理回归各自范围分别记录，不将合成scan的规划测试称作物理导航。

## 最终四例单次物理冒烟

campaign：`w11r1-repair-smoke-v10`。每例均为REPAIR_SHA；未改变原案例，未挑选同版本重复运行。

| 案例 | run ID | 真值位置误差m | 偏航误差° | 任务时间s | 修正次数 |
| --- | --- | --- | --- | --- | --- |
| C01 | `20261015T090302Z_w11r1-repair-smoke-v10_C01_s2000_2b2382` | 0.087368 | 11.195169 | 46.42 | 0 |
| C09 | `20261015T090450Z_w11r1-repair-smoke-v10_C09_s2008_ed05e9` | 0.066089 | 4.115069 | 40.22 | 0 |
| B04 | `20261015T090054Z_w11r1-repair-smoke-v10_B04_s3003_ac23a9` | 0.057441 | 9.925252 | 55.36 | 0 |
| B06 | `20261015T085752Z_w11r1-repair-smoke-v10_B06_s3005_c1366a` | 0.079528 | 3.452397 | 101.14 | 0 |

四例均SUCCESS、action SUCCEEDED、冻结2s联合稳定窗口通过；环境碰撞0、障碍接触0、非足触地0、跌倒0。每例完整scene/map/参数/checkpoint哈希、启动检查、action记录、安全事件、退出及回收状态见`*-result.json`和`smoke-summary.json`，原始CSV和launch日志留在ignored raw/run_data。不是W13正式统计，未运行formal-v2或RTF验收。

## 取消与目标替换

最终run：`20261015T090629Z_dev_cancel_s0_3a8460`，被测REPAIR_SHA，结果`PASS`。真实取消状态5、旧目标替换状态6、新目标状态4；策略零延迟0.020000s，停车位移0.107777m≤.453m；重新arm前不运动，命令图和TF检查通过。碰撞/跌倒0，无遗留进程。最终五次运行各17个launch子进程全部正常退出，无异常退出行。

附加到达核验也通过：0.5s速度窗口满足<0.05并连续保持2s，真值位置误差0.126661m、偏航误差1.703664°。原F09接口回归及附加到达结果均保留在JSON。

开发期首次取消脚本因旧记录器名称w11_evaluator与现代t1_evaluator不兼容报告三项图FAIL，原始文件保留；仅W11R1入口适配实际订阅者名称，离线同数据21+3检查通过，4种非法发布/订阅负控均拒绝。W13评测器未改、正式失败未重分类。

## 开发失败与已知风险

所有H1–H13尝试见experiments、development-run-index及raw；没有用成功替换失败。曾拒绝：manager关闭exit−6（已用pre-shutdown和真实bond测试修复）、分段回弹耗尽、前移预目标、提前GoalAngle成本、纯Dubin无路、格点小转向归零、格点终点yaw量化、MPPI仅加朝向成本仍超时。最终不含被撤除的行为。

这仍只是窄范围冒烟，不能证明可靠率。新全局规划器、RPP路径控制、连续目标yaw适配和低转向意图阈值需要独立W11/W12回归；小命令噪声、墙边可达性、反向目标、回弹和负载下RPC响应仍是重点。有限失败停车不等于导航成功。Twist本身没有UUID，代次保护依赖Nav2 action状态和时间戳，独立验证须观察真实抢占。

原formal-v1 S06及开发H7 B04均存在command_gate在SIGINT后rclpy订阅析构异常exit1；不是本次新manager的已修复exit−6。本任务未改W10来掩盖它，逐run异常退出行公开保存。理想truth odom/姿态补偿等既有仿真假设仍在；导航控制没有读取独立评测真值来纠正终点。

## 第二部分独立验证矩阵（本部分未执行）

1. 先只读检查分支、HEAD、dirty、REPAIR到HANDOFF差异、原始证据哈希、进程和端口；确认源码/参数/测试入口与REPAIR一致，再构建/包测试。
2. 10次全新启动：lifecycle顺序、initialpose确认、map→odom、真实action/service响应、明确有限失败/退出；不共享已有ROS图来冒充冷启动。
3. C01、C09、B04、B06各3次，固定原场景/地图/seed/事件/目标/120s及到达判据；全部记录首次结果，不能删除失败。
4. 完整W11 S01–S05，取消/重新arm/目标替换及命令链/TF检查；取消的额外到达稳定结果单列。
5. W12完整回归：crossing/blocking/approach各原3例及静态S03；实体持续、真实绕行、scan/costmap、stop/恢复和安全统计保留原标准。
6. 启动/endpoint缺失、迟到ACK、服务不可响应、ABORTED0、定位缺失、过冲/回弹、纠偏耗尽、旧目标迟到命令故障检查；核对自动化与外部观测。
7. 全部结束进程/端口审计，报告所有异常退出与未证明项。第二部分只可判候选最终PASS/FAIL；若需源码修复，退回修复阶段、新SHA和新证据，不在验证中调参。

第二部分不得修改`src/`、`scripts/`、本候选测试入口、配置/launch/地图/案例/BT、外部原语/checkpoint/upstream、原W11/W12/W13证据及报告，尤其用户`docs/start_commands.md`。只在新的独立证据目录写结果。旧W11/W12脚本默认OUT指向历史目录，必须像本入口一样绑定新的T1_EVAL_OUT并核对，不可直接覆盖运行。

W13 formal-v2、RTF验收以及W14/W15不属于本部分或自动启动项。独立验证通过后，仍需用户分派W13重验；本交接不授权自动推进。

## 环境、进程与第一条命令

最终本任务进程和DDS端口状态见`process-audit.json`；所有自持进程已退出。只允许关闭验证器自己持有的进程组，不kill无关ROS/Python。

第二部分第一条命令必须只读：

```bash
cd /home/haoyang/humanoid_nav_ws && git status --short --branch && git log -8 --oneline --decorate
```

核对后再进入环境：

```bash
export T1_VENV=/home/haoyang/venvs/humanoid_mujoco
export T1_UPSTREAM=/home/haoyang/robot_sim_sources/booster_deploy
export T1_LOCAL_DDS=1 T1_DDS_PORT=42124 ROS_DOMAIN_ID=34
source scripts/env.sh
```

系统colcon构建；仿真入口明确`"$T1_VENV/bin/python"`。当前结果复现命令形态为`"$T1_VENV/bin/python" docs/evidence/W11R1/smoke.py 新campaign C01,C09,B04,B06`和`... cancel_smoke.py 新取消目录`。禁止复用最终campaign覆盖结果。离线取消审计可用`cancel_smoke.py --audit-result <已有result.json>`，它创建旁路audited文件，不启动仿真或覆盖原文件。

外部冻结：booster_deploy `7bb1462e2ac742eba5c01659b815e2e99fbe617f`，booster_assets `12a97516f57469078707afb587a5f84c1cb110d1`；本机Jazzy Smac1.3.13原语路径与哈希在freeze-audit。不得重装环境、改.bashrc、unset PYTHONPATH或升级依赖来掩盖集成错误。

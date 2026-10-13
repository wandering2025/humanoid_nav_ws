# W03 ROS包骨架与启动环境

难度：L3 中等。估算投入：2–3小时，含调试与验证，不是保证。
依赖：W02。覆盖需求：R6、R7。遵循决策：D1、D2、D8。
初始状态：TODO；实时状态以[执行看板](执行看板.md)和[本步记录](records/W03.md)为准。
工作类型：用户手动分派后执行本步。一次只做本步，不自动开始后续步骤。

## 进入本步

先读[总规划](总规划.md)、[执行规则](执行规则.md)、[环境定位](环境定位.md)、[当前交接](交接记录.md)、[证据与参考](证据与参考.md)和本步记录。
确认依赖步骤DONE及其代码SHA、验收证据有效。先检查Git差异，保留已有工作；把本步状态置IN_PROGRESS并记录时间、分支、起始SHA。
所有$WS/$PLAN等路径按环境定位解析。本文件中的目标路径是要创建或更新的产物，不代表已存在。
## 目标和产物

创建可构建、可测试、可在正确解释器下运行的工程骨架。包名固定为 t1_mujoco、t1_description、t1_navigation、t1_bringup。
产物：package.xml、相应构建配置、Python模块/资源标记、最小diagnostic入口、启动骨架、README、环境入口和$WS/AGENTS.md。

## C1：按契约建立目录

t1_mujoco使用ament_python；资源型description/navigation使用ament_cmake；bringup若承载后续Python评测可采用ament_python，在本步固定。
只创建本步所需文件。未实现功能写明确TODO并默认禁用，不写“成功”假返回。
把 docs/architecture 中的契约链接进 README；根目录 AGENTS.md 写明本规划目录、Git 提交和串行执行规范。
依赖只声明实际需要的包。新依赖经本地检查后按需安装，禁止泛化升级。

## C2：处理解释器与外置依赖

1. 对比系统Python与venv Python版本、rclpy及NumPy可导入情况。
2. 检查colcon自身解释器和生成console_script的shebang。选择可复现的venv构建方式或显式venv启动包装，写明理由。
3. 创建一个不启动物理的诊断入口，实际打印sys.executable、ROS发行版、mujoco/torch版本、资产路径。
4. 通过 ros2 run 和 ros2 launch 各跑一次该入口，证明真实节点进程能同时加载三类库。单独python -c成功不算通过。
5. booster_deploy可能不是可pip安装的标准项目，先看是否有打包元数据；不得盲目pip install -e。需要添加导入路径时只在仿真进程封装并从配置解析，不污染全局.bashrc。
6. 使用UPSTREAM/VENV等外部路径参数或被忽略的本机配置，代码不硬编码用户名。

## C3：构建和最小测试

执行 colcon list，确认四包且没有意外扫描上游仓库。
在选定环境构建，检查安装后的launch/config资源可以找到。
单元测试聚焦参数错误、路径不存在、依赖缺失的可读报错；不为纯目录存在堆大量测试。
为W04预留明确入口但不能声称已完成仿真。清理自己产生的临时进程。

## C4：验收

- 四包被colcon识别，构建成功，测试无失败且测试数非误判为0。
- 从$WS以外的工作目录调用诊断也成功，不能依赖cd上游的sys.path.append(".")。
- 上游仓库未修改；环境启动脚本不会重写.bashrc或自动升级。
- 日志包含真实解释器路径及版本，启动文件退出码为0。
- 记录具体构建命令和下次进入环境的最短流程。

## 边界与失败处理

不实现LiDAR/SLAM/Nav2。若venv与apt包存在ABI冲突，先最小import复现并查明，不通过复制所有site-packages或sudo pip规避。

## 记录、版本控制和中断恢复

检查点C1、C2、C3、C4分别完成后立即更新[本步记录](records/W03.md)，保存已完成/正在做/未完成、命令、退出码、实测值、证据和代码SHA。
原始日志与截图放$WS/docs/evidence/W03/，大文件按执行规则忽略；小型结果摘要需纳入代码Git。
建议本步完成提交主题：`feat(W03): scaffold ROS packages and runtime entrypoints`。未完成但需要交接时用`wip(W03): describe checkpoint`，并明确未通过项。
先提交代码/证据，再填写代码SHA并提交规划记录。完成后更新看板和交接，不只在聊天中汇报。
中断时至少记录下一条可执行命令、cwd、解释器/环境、脏文件、仍在运行的进程、最近失败原因及不应重复的操作。若仍有安全相关仿真进程，先进入停止状态。
完成状态要求所有本步验收通过；不能以“代码写完”“暂时跑不起来”标DONE。

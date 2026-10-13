# W01 环境基线与版本冻结

难度：L2 常规。估算投入：1–2小时，含调试与验证，不是保证。
依赖：无。覆盖需求：R6、R7。遵循决策：D1、D8。
初始状态：TODO；实时状态以[执行看板](执行看板.md)和[本步记录](records/W01.md)为准。
工作类型：用户手动分派后执行本步。一次只做本步，不自动开始后续步骤。

## 进入本步

先读[总规划](总规划.md)、[执行规则](执行规则.md)、[环境定位](环境定位.md)、[当前交接](交接记录.md)、[证据与参考](证据与参考.md)和本步记录。
确认依赖步骤DONE及其代码SHA、验收证据有效。先检查Git差异，保留已有工作；把本步状态置IN_PROGRESS并记录时间、分支、起始SHA。
所有$WS/$PLAN等路径按环境定位解析。本文件中的目标路径是要创建或更新的产物，不代表已存在。
## 目标和产物

保全已验证环境，建立可回退代码基线，记录真实版本与启动方式。产物：
- $WS/docs/baseline/environment.md、python-packages.txt、apt-packages.txt、assets-manifest.md。
- $WS/dependencies.repos，包含 deploy 和 assets 两个仓库的真实SHA。
- 代码仓库环境基线提交及开发分支。
- 本步记录与看板更新。

## 开始前读取

$WS/SETUP_REPORT.md、.gitignore、仓库内适用 AGENTS.md；本规划的证据与参考。
先确认 Ubuntu-24.04、whoami、pwd、Git根；Windows默认 Ubuntu 不可直接当作项目环境。

## C1：核实与保护现状

1. 检查 $WS、$UPSTREAM、$ASSETS 的 status 和 log。用户新增代码不在本规划观察范围内，若已存在则记录并适配，不覆盖。
2. 记录系统/Python/ROS/MuJoCo/PyTorch/NumPy/onnxruntime 版本、解释器路径、venv配置。
3. 检查 ROS 与 venv 共存：显式 source ROS 后激活 venv，实际导入 rclpy/mujoco/torch。记录输出。
4. 读取低速死区、横漂、GUI退出问题的原始报告段落，标注“历史结果，尚未重测”。

## C2：建立依赖清单

1. 分别保存两个官方仓库 remote、SHA、dirty状态、资产和策略实际路径。
2. 对当前 checkpoint 和主 MJCF计算SHA256，记录模型自由度、名称、文件大小、许可证位置。无需遍历每个大型mesh。
3. Python清单使用当前解释器的 python -m pip freeze；apt仅记录相关安装包，不导出整机私密配置。
4. dependencies.repos 使用实际SHA，不用 main；现有外置路径继续使用，不重复clone。
5. venv的 editable本机路径不能被当成可跨机器安装的完整锁文件。分别说明环境快照与可复现依赖来源。

## C3：Git基线

1. 审查 .gitignore，补充 *.mcap、记录目录、venv和缓存排除；地图和小型CSV应仍可追踪。
2. 若无历史，先提交原环境报告、截图、忽略规则，使用 docs: record verified environment baseline。
3. 在已确认干净基线创建 feat/t1-mujoco-navigation；若已存在该分支则读取后复用，不强制重建。
4. 提交本步新增版本清单，记录完整SHA。不要触碰上游仓库历史和源文件。

## C4：验收与交接

- 命令能从正确发行版重复执行；联合import成功。
- 两个上游SHA可追溯，dirty状态与开始一致。
- 没有提交token、venv、大型策略副本和缓存。
- 工作区有基线commit与开发分支；清单里的路径确实存在。
- 把尚未复测的行走问题交给W05，不把它们写成“已解决”。

## 边界与失败处理

本步不升级、不重装、不创建ROS实现。缺包先记录缺少的精确包及谁需要，不安装一整套新环境。
若Git无身份按执行规则使用仓库局部自动化身份。依赖SHA与历史报告不同则记录差异，不能强制checkout覆盖用户改动。

## 记录、版本控制和中断恢复

检查点C1、C2、C3、C4分别完成后立即更新[本步记录](records/W01.md)，保存已完成/正在做/未完成、命令、退出码、实测值、证据和代码SHA。
原始日志与截图放$WS/docs/evidence/W01/，大文件按执行规则忽略；小型结果摘要需纳入代码Git。
建议本步完成提交主题：`chore(W01): freeze verified environment baseline`。未完成但需要交接时用`wip(W01): describe checkpoint`，并明确未通过项。
先提交代码/证据，再填写代码SHA并提交规划记录。完成后更新看板和交接，不只在聊天中汇报。
中断时至少记录下一条可执行命令、cwd、解释器/环境、脏文件、仍在运行的进程、最近失败原因及不应重复的操作。若仍有安全相关仿真进程，先进入停止状态。
完成状态要求所有本步验收通过；不能以“代码写完”“暂时跑不起来”标DONE。

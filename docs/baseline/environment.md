# W01 本机环境基线

采集时间：2026-10-13T19:31:21.740721+00:00（UTC；Europe/Berlin 为 +02:00）。工作目录固定 /home/haoyang/humanoid_nav_ws。

## 实测值

- 用户 haoyang；WSL_DISTRO_NAME=Ubuntu-24.04；Ubuntu 24.04.5 LTS / noble；内核 6.18.54.1-microsoft-standard-WSL2。
- Python 3.12.3；解释器 /home/haoyang/venvs/humanoid_mujoco/bin/python；base_prefix=/usr；venv include-system-site-packages=false。
- ROS Jazzy；rclpy 来自 /opt/ros/jazzy/lib/python3.12/site-packages；apt 精确版本见 apt-packages.txt。
- MuJoCo 3.15.0；PyTorch 运行时 2.14.1+cu130（distribution metadata/pip freeze 为 2.14.1）；NumPy 2.5.4；onnxruntime 1.31.0；booster-assets 1.0.0（editable 外置源码）。
- 当前进程 torch.cuda.is_available()=False，历史报告为 True，未定位差异原因；官方策略按 CPU 运行，CPU 导入与模型加载通过，不以此宣称 CUDA 已验证。
- ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST；ROS_DOMAIN_ID 未设置（默认 0）。联合导入 rclpy/mujoco/torch 成功。
- 两个上游仓库 SHA 与历史报告一致、clean，路径/哈希/许可证详见 assets-manifest.md；dependencies.repos 固定这两个真实 commit。

## 正确终端与复核

Windows 入口：

```powershell
wsl -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws
```

在此工作目录运行（不安装、不升级）：

```bash
source /opt/ros/jazzy/setup.bash
if test -f /home/haoyang/humanoid_nav_ws/install/setup.bash; then
  source /home/haoyang/humanoid_nav_ws/install/setup.bash
fi
source /home/haoyang/venvs/humanoid_mujoco/bin/activate
python docs/baseline/verify_baseline.py
python -m pip freeze
```

上游官方 GUI 命令（W01 未运行）：

```bash
python /home/haoyang/robot_sim_sources/booster_deploy/scripts/deploy.py --task t1_walk --mujoco --device cpu
```

此命令通过官方 stdin 输入 x/y/yaw，Space 暂停/继续。W01 不运行行走或 GUI；W03 必须验证实际 colcon/launch 可执行解释器，本次 import 不替代该验收。不要 unset PYTHONPATH。

## 快照与跨机器复现边界

python-packages.txt 是 ROS source 后当前解释器的 `python -m pip freeze` 原样快照，因 ROS PYTHONPATH 含 apt 提供的 ROS Python 包，不能直接整体 `pip install -r` 到新机器。editable booster-assets 包含本机绝对路径，只描述本机状态。apt-packages.txt 仅收集 installed 的 ROS/Python/构建/图形相关包，不是整机完整镜像。

可复现来源：Ubuntu 24.04 / ROS Jazzy 的 apt 包版本清单、dependencies.repos 固定官方仓库 SHA、deploy/requirements.txt 与 Python 快照对照、assets/pyproject.toml 和 editable 安装来源、策略/MJCF SHA256。上游 requirements 未固定版本，本快照没有 wheel 哈希或软件镜像，不保证未来仓库仍提供同一 wheel。不要把上述文件宣称为完整跨机器锁文件。当前继续使用既有外置目录，不 clone、不 checkout、不安装。将来 vcstool 导入清单的目标父目录为 /home/haoyang/robot_sim_sources；本步未执行导入。

## 历史报告与未复测事项

SETUP_REPORT.md §16.6/16.10：vx=0.2/0.3 不能持续行走；vx=0.5 时约 0.39m/s；9.5s 仿真前进中横漂约 0.5m、偏航约 +4°；横移小命令无响应；GUI 实时系数约 0.7–0.85；viewer 关闭后偶发退出段错误。全部是历史结果，W01 尚未重测，不称已解决。运动标定交 W05，viewer/无界面运行交 W04。

历史日志位于 /home/haoyang/t1_walk_evidence/{run3_monitor.log,run4_monitor.log,run5_monitor.log}。既有 Gazebo/RViz 截图仅为历史画面，本步已检查文件与画面，不构成新 GUI 验收。未重装基础环境、未创建 ROS 实现、未更改上游。

导入期间 onnxruntime 报遥测 device ID 无法持久化，pip 报缓存不可写并禁用缓存，两条警告均未阻止命令成功；未修改外部缓存权限。

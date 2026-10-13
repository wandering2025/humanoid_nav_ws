> W01 说明（2026-10-13）：以下为既有启动说明。当前版本与复核以 docs/baseline/environment.md 为准；工作目录仅确定开发上下文，工具权限由 sandbox 控制。后续 ROS launch 尚未实现。

对，现在不需要让 Codex 或 DeepSeek继续安装东西。环境已经达到开发条件。继续追加 setup 只会增加版本冲突风险。

## 在哪里开发

不需要再新建项目文件夹。现有目录就是项目根目录：

```text
/home/haoyang/humanoid_nav_ws
```

你自己的代码以后全部放在：

```text
/home/haoyang/humanoid_nav_ws/src
```

Booster 官方代码继续单独保留：

```text
/home/haoyang/robot_sim_sources/booster_deploy
```

结构关系是：

```text
/home/haoyang/
├── humanoid_nav_ws/                 # 你的项目
│   ├── src/                         # 你的 ROS 包
│   ├── build/
│   ├── install/
│   ├── log/
│   ├── docs/
│   └── SETUP_REPORT.md
│
├── robot_sim_sources/
│   └── booster_deploy/              # 官方依赖，尽量不修改
│
└── venvs/
    └── humanoid_mujoco/             # Python 环境
```

不要再创建类似下面这样的重复目录：

```text
~/t1_project
~/mujoco_project
~/robot_navigation
```

这会让工作区、Git 仓库和依赖关系变乱。

## 每次开始开发

### 第一步：从 Windows 进入正确的 Ubuntu

在 PowerShell 或 Windows Terminal 中：

```powershell
wsl -d Ubuntu-24.04 --cd /home/haoyang/humanoid_nav_ws
```

因为默认 `wsl` 目前会进入另一个 `Ubuntu`，所以不要省略 `-d Ubuntu-24.04`。

### 第二步：打开 VS Code

进入 WSL 后：

```bash
cd ~/humanoid_nav_ws
code .
```

确认 VS Code 左下角显示类似：

```text
WSL: Ubuntu-24.04
```

Codex 或 DeepSeek 也应该从这个目录启动，而不是从 `booster_deploy` 启动：

```bash
cd ~/humanoid_nav_ws
```

这样默认开发上下文是项目工作区；官方仓库按执行规则保持只读，不以工作目录作为访问隔离。

## 不同终端运行什么

### 终端一：普通 ROS 开发

```bash
source /opt/ros/jazzy/setup.bash
source ~/humanoid_nav_ws/install/setup.bash
cd ~/humanoid_nav_ws
```

检查：

```bash
echo "$ROS_DISTRO"
ros2 topic list
```

应该看到：

```text
jazzy
```

如果 `.bashrc` 已经自动加载，这些 `source` 可能可以省略。但开发脚本和故障排查时显式执行更可靠。

### 终端二：MuJoCo 和 Booster

这个终端需要同时加载 ROS 和 Python venv：

```bash
source /opt/ros/jazzy/setup.bash
source ~/humanoid_nav_ws/install/setup.bash
source ~/venvs/humanoid_mujoco/bin/activate

cd ~/robot_sim_sources/booster_deploy
```

运行现有策略：

```bash
python scripts/deploy.py \
  --task t1_walk \
  --mujoco \
  --device cpu
```

快速检查组合环境：

```bash
python -c "import rclpy, mujoco, torch; print('environment OK')"
```

### 终端三：RViz 和导航

现在还没有导航包，所以暂时只需要：

```bash
source /opt/ros/jazzy/setup.bash
source ~/humanoid_nav_ws/install/setup.bash

rviz2
```

以后项目建立后，这个终端将运行：

```bash
ros2 launch t1_bringup mapping.launch.py
```

或者：

```bash
ros2 launch t1_bringup navigation.launch.py
```

这些 launch 文件目前还不存在。

## 什么时候需要编译

不需要每次打开终端都执行 `colcon build`。

只有以下情况才需要：

1. 新建了 ROS 包
2. 修改了 `package.xml`
3. 修改了 `CMakeLists.txt`
4. 添加了新的可执行节点
5. 修改了消息或服务定义
6. 修改了 C++ 代码

编译命令：

```bash
cd ~/humanoid_nav_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
```

增加依赖以后再执行：

```bash
cd ~/humanoid_nav_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -r -y
```

这也不是每次启动都需要运行。

## 当前阶段唯一值得做的收尾

如果还没提交环境基线，可以执行：

```bash
cd ~/humanoid_nav_ws

git add .gitignore SETUP_REPORT.md docs
git commit -m "docs: record verified development environment"
```

除此之外，不要再让 DeepSeek做进一步安装。

当你决定正式开工时，第一项开发任务才是让它在：

```text
~/humanoid_nav_ws/src
```

中创建最小的 `t1_mujoco` ROS 包。当前保持 `src` 为空完全正常。
# 机器人仿真开发环境配置报告 — Ubuntu 24.04 + ROS 2 Jazzy

最终版本: 2026-10-13（含第二轮修复与验收）
配置者: Codex（在既有 Ubuntu 26.04 发行版内驱动，全程未修改该发行版）
说明: 本报告不含任何密码、令牌或 API Key

---

## 1. 最终结论：成功

三项关键验收**全部通过**：

| 验收项 | 结果 |
| --- | --- |
| `haoyang` 的 sudo | **通过**（交互式 `sudo -v` → `SUDO_OK`；无 NOPASSWD） |
| WSLInterop | **通过**（由 WSL 正常启动流程注册，未手工注册/未用 user namespace） |
| 硬件 OpenGL | **通过**（Gazebo、RViz2 实测走 D3D12 → NVIDIA RTX 4050，GPU 利用率实测上升） |

基础技术栈（Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / ros_gz / RViz2 / Nav2 /
SLAM Toolbox / colcon / rosdep / Git）全部安装并逐项验证通过。

硬件渲染需显式选择 Gallium D3D12 驱动（原因见第 4 节）；该设置已按用户批准固化到
`/home/haoyang/.bashrc`，并已在真实应用上复验通过。**未**使用第三方 Mesa PPA，
**未**在 WSL 内安装 NVIDIA Linux 内核驱动。

## 2. 主机与 WSL 环境

| 项目 | 更新前 | 更新后 |
| --- | --- | --- |
| WSL 版本 | 2.7.14.0 | **3.0.3.0** |
| WSL 内核 | 6.18.33.2-2 | **6.18.54.1-1** |
| WSLg 版本 | 1.0.73.2 | **1.0.79** |
| MSRDC | 1.2.7214 | 1.2.7391 |
| Direct3D / DXCore | 1.611.1-81528511 / 10.0.26100.1 | 同左（未变） |
| Windows | 10.0.26200.9457 | 同左 |
| WSL 发行版 | `Ubuntu`(26.04，未改动) + `Ubuntu-24.04`(本次新建) | 同左 |
| `.wslconfig` | `[wsl2] networkingMode=mirrored` / `dnsTunneling=true` / `autoProxy=true`（只读查看，未修改） | 同左 |

更新操作由用户在 Windows PowerShell 中执行：`wsl --update` → `wsl --shutdown` →
`wsl -d Ubuntu-24.04 -- true`。未执行 `wsl --unregister`，未删除/注销任何发行版。

## 3. Ubuntu 24.04 基础状态

| 项目 | 实测值 |
| --- | --- |
| 发行版 | Ubuntu 24.04.5 LTS（VERSION_ID 24.04 / CODENAME noble） |
| 内核 | 6.18.54.1-microsoft-standard-WSL2 |
| PID 1 | systemd（`/etc/wsl.conf` 中 `[boot] systemd=true`） |
| WSLg | `/mnt/wslg` 存在；`DISPLAY=:0`、`WAYLAND_DISPLAY=wayland-0`、`XDG_RUNTIME_DIR=/run/user/1000` |
| 默认用户 | `haoyang`（uid/gid 1000，组 `haoyang`,`adm`,`sudo`），`HOME=/home/haoyang` |
| 密码 | 已由用户设置（`passwd -S` = `P`；**本报告不记录密码**） |
| 内存 | 15 GiB（可用约 12 GiB），Swap 4 GiB |
| 根文件系统 | `/dev/sde` ext4，1007 GB 总量，约 949 GB 可用 |
| 语言环境 | `LANG=LC_ALL=en_US.UTF-8`（`/etc/default/locale`） |

## 4. GPU 与图形栈（关键项）

| 项目 | 实测值 |
| --- | --- |
| GPU | NVIDIA GeForce RTX 4050 Laptop GPU（6141 MiB）；另有 Intel Arc Pro 核显 |
| Windows 驱动 | 596.58（WSL 内可见版本号 595.78 / CUDA 13.2） |
| CUDA 通道 | `nvidia-smi` 正常 |
| 设备节点 | `/dev/dxg` 存在；**更新后 `/dev/dri` 也已出现**（`card0`、`renderD128`） |
| D3D12 用户态组件 | `/usr/lib/wsl/lib/libdxcore.so`、`libd3d12.so`、`libd3d12core.so` 均在 ld.so 缓存中 |
| Mesa | 发行版自带 25.2.8（`libgl1-mesa-dri`/`libegl-mesa0`/`libglx-mesa0`），含 `d3d12_dri.so`；无第三方 PPA |

### 4.1 默认渲染 vs 显式选择 D3D12（更新后实测）

| 命令 | OpenGL vendor | OpenGL renderer | Accelerated | GL |
| --- | --- | --- | --- | --- |
| `glxinfo -B` | Mesa | llvmpipe (LLVM 20.1.2, 256 bits) | no | 4.5 |
| `GALLIUM_DRIVER=d3d12 glxinfo -B` | Microsoft Corporation | D3D12 (Intel(R) Arc(TM) Pro Graphics) | **yes** | 4.6 |
| `GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA …` | Microsoft Corporation | D3D12 (NVIDIA GeForce RTX 4050 Laptop GPU) | **yes** | 4.6 |

**根因说明**：GPU 通道本身正常（`/dev/dxg` + WSL 库 + Mesa d3d12 驱动齐备），
llvmpipe 只是 Mesa 的**默认驱动选择**结果 —— WSLg 的 Xwayland 未暴露 DRI3 设备，于是
回落到软件光栅化。显式指定 Gallium 驱动后硬件渲染立即生效，无需 PPA、无需安装
NVIDIA Linux 驱动、无需修改 `.wslconfig`。

### 4.2 固化设置（用户批准：使用独显）

`/home/haoyang/.bashrc` 追加（带注释）：

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA GeForce RTX 4050 Laptop GPU"
```

复验：新开的交互式 shell 中 `glxinfo -B` → `D3D12 (NVIDIA GeForce RTX 4050 Laptop GPU)`、
`Accelerated: yes`、GL 4.6。

注：该变量随交互式 shell 生效（Windows Terminal → Ubuntu 24.04 的正常使用路径）。
若日后需要从非交互入口（例如 `wsl.exe -d Ubuntu-24.04 -- rviz2`）启动图形程序，
需改为写入 `/etc/environment` 或 `wsl.conf` 的 `[boot] command`。

## 5. ROS 2 Jazzy

| 项目 | 实测值 |
| --- | --- |
| `ROS_DISTRO` / ros2 路径 | `jazzy` / `/opt/ros/jazzy/bin/ros2` |
| 软件源 | `ros2-apt-source 1.3.0~noble`，`packages.ros.org/ros2/ubuntu`，`Suites: noble` |
| 主要包 | `ros-jazzy-desktop 0.11.0-1noble.20261006.040109`、`ros-dev-tools 1.0.3`（共 402 个 ros-jazzy-*） |
| `ros2 doctor --report` | 无阻断错误；rmw_fastrtps_cpp；distribution active |
| rosdep | `20-default.list` 已初始化，缓存已为用户 haoyang 生成 |
| DDS 通信 | talker/listener 连通（收到 9 条 Hello World，`/chatter` 1 pub / 1 sub） |
| DDS 发现 | 因 `networkingMode=mirrored` 组播失效，已固定 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`（单机模式） |

## 6. Gazebo Harmonic 与 ros_gz

| 项目 | 实测值 |
| --- | --- |
| Gazebo | Harmonic，`gz sim --versions` = **8.15.0** |
| 来源 | `ros-jazzy-ros-gz 1.0.24-1noble.20261006.041805`（官方 ROS 源携带的 gz vendor 包，16 个 vendor 包） |
| ros_gz 包 | `ros_gz`、`ros_gz_bridge`、`ros_gz_image`、`ros_gz_interfaces`、`ros_gz_sim`、`ros_gz_sim_demos`；prefix 均为 `/opt/ros/jazzy` |
| GUI 复验（硬件渲染） | `gz sim -v 4 shapes.sdf` 打开，shapes 场景（地面网格 + box/cylinder/sphere/capsule/ellipsoid/cone + 阴影）正常显示；渲染期间 GPU 利用率 6% → **37%**、显存 1313 → 1519 MiB；无致命错误；关闭后无遗留进程 |
| 未安装 | Gazebo Classic、Fortress、Ionic、Jetty 均未安装 |

## 7. RViz2

| 项目 | 实测值 |
| --- | --- |
| 可执行 / 配置 | `/opt/ros/jazzy/bin/rviz2` + `/opt/ros/jazzy/share/rviz_common/default.rviz` |
| 窗口 | `... - RViz` 1200x846 正常打开 |
| 渲染 | `OpenGl version: 4.6 (GLSL 4.6)`，默认网格正常显示，31 fps，GPU 利用率约 24% |
| 日志 | 无致命 OpenGL 错误（仅 `Stereo is NOT SUPPORTED` 提示） |
| 退出 | 正常关闭，无遗留进程 |

## 8. Nav2 与 SLAM Toolbox

| 项目 | 实测值 |
| --- | --- |
| navigation2 | 1.3.13-1noble.20260921.033624 |
| nav2_bringup | 1.3.13-1noble.20261006.042503；prefix `/opt/ros/jazzy` |
| slam_toolbox | 2.8.5-1noble.20260915.204151；prefix `/opt/ros/jazzy` |
| nav2_map_server 可执行 | map_server、map_saver_cli、map_saver_server、costmap_filter_info_server |
| slam_toolbox 可执行 | async / lifelong / localization / map_and_localization_slam_toolbox_node 等 |
| 附加工具 | tf2-tools 0.36.23、rqt_tf_tree 1.1.1、teleop_twist_keyboard 2.4.1 |

## 9. 工作区

| 项目 | 实测值 |
| --- | --- |
| 路径 | `/home/haoyang/humanoid_nav_ws`（含 `src/`、`build/`、`install/`、`log/`） |
| 文件系统 | `/dev/sde` ext4（不在 `/mnt/c`、`/mnt/d` 等 Windows 挂载点） |
| Git | 已初始化（master，尚无提交），`.gitignore` 含 `build/`、`install/`、`log/`、`*.bag`、`*.db3`、`__pycache__/`、`.vscode/`、`.cache/` |
| colcon | `colcon build --symlink-install` 成功，`install/setup.bash` 已生成 |
| shell 集成 | `~/.bashrc` 中已（去重）加入：ROS 2 setup、DDS 发现范围、workspace overlay、D3D12 渲染 |

## 10. 关键命令（安装与验收）

```bash
# 发行版（Windows 侧）
wsl --install -d Ubuntu-24.04 --no-launch
wsl -d Ubuntu-24.04 -- whoami ; wsl -d Ubuntu-24.04 -- bash -lc "sudo -v && echo SUDO_OK"
wsl --update ; wsl --version ; wsl --shutdown ; wsl -d Ubuntu-24.04 -- true

# 基础工具与语言环境（Ubuntu-24.04 内）
apt-get update && apt-get upgrade -y
apt-get install -y locales curl ca-certificates software-properties-common git build-essential
locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

# 官方 ROS 2 源与 ROS 2 Jazzy
curl -L -o /tmp/ros2-apt-source_1.3.0.noble_all.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb
dpkg -i /tmp/ros2-apt-source_1.3.0.noble_all.deb && apt-get update
apt-get install -y ros-jazzy-desktop ros-dev-tools
rosdep init && rosdep update

# Gazebo Harmonic + 桥接 + 导航/建图
apt-get install -y ros-jazzy-ros-gz
apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-tools ros-jazzy-rqt-tf-tree

# 验收诊断工具（额外，非技术栈要求）
apt-get install -y mesa-utils imagemagick

# 工作区
mkdir -p ~/humanoid_nav_ws/src && cd ~/humanoid_nav_ws && git init
colcon build --symlink-install

# 验收
glxinfo -B ; eglinfo -B
env GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA GeForce RTX 4050 Laptop GPU" glxinfo -B
gz sim -v 4 shapes.sdf ; rviz2
```

## 11. 修改过的配置文件与系统状态

| 对象 | 变更 |
| --- | --- |
| 新用户 / 目录 | 用户 `haoyang`(uid 1000, 组 sudo+adm)、`/home/haoyang/humanoid_nav_ws`（sudo 密码由用户自行设置，未记录） |
| `/etc/wsl.conf` | 新增 `[user] default=haoyang`（原 `[boot] systemd=true` 保留） |
| `/etc/default/locale` | `LANG`/`LC_ALL` = `en_US.UTF-8`（`update-locale` 写入） |
| `/etc/apt/sources.list.d/ros2.sources` | 由 `ros2-apt-source 1.3.0~noble` 安装（指向官方 packages.ros.org，Suites: noble） |
| `/etc/ros/rosdep/sources.list.d/20-default.list` | `rosdep init` 生成 |
| `/home/haoyang/.bashrc` | 追加：`source /opt/ros/jazzy/setup.bash`；`export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`；`source ~/humanoid_nav_ws/install/setup.bash`；`export GALLIUM_DRIVER=d3d12`；`export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA GeForce RTX 4050 Laptop GPU"`（均做去重） |
| `/home/haoyang/humanoid_nav_ws/.gitignore` | 新建 |
| Windows 侧 | 执行 `wsl --update`（WSL 2.7.14.0 → 3.0.3.0）。**未**改动 `.wslconfig`，**未**改注册表，**未**卸载/降级显卡驱动 |

未修改：既有 Ubuntu 26.04 的任何配置、`/etc/environment`、`/etc/profile*`、任何磁盘或目录结构。

## 12. WSLInterop 验收详情

| 检查 | 实测 |
| --- | --- |
| `/proc/sys/fs/binfmt_misc/WSLInterop` | 存在 |
| 状态 / interpreter / magic / flags | `enabled` / `/init` / `4d5a`(MZ) / `PF` |
| 注册方式 | WSL 正常启动流程（`systemd-binfmt.service` active；**未**手工写 `register`，**未**使用 user namespace） |
| Windows 程序 | `cmd.exe /c ver` → `Microsoft Windows [版本 10.0.26200.9457]`；`powershell.exe` → `5.1.26100.9444` |
| 持久性 | `wsl --update` + `wsl --shutdown` 之后，`Ubuntu-24.04` 与 `Ubuntu` 两个发行版的 binfmt namespace 中均已自动注册，重启后依然有效 |

## 13. 仍存在的警告与限制

1. **默认驱动选择仍为 llvmpipe**：这是 Mesa 在 WSLg（Xwayland 未暴露 DRI3）下的默认行为，
   不是故障。已通过 `GALLIUM_DRIVER=d3d12` 显式指定解决并固化；若今后某程序仍显示
   llvmpipe，检查其是否在交互式 shell（或继承该环境变量）中启动。
2. **DDS 限于单机**：`networkingMode=mirrored` 下组播发现失效，已固定
   `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`；跨主机通信需改用 `ROS_STATIC_PEERS`
   或 discovery server。
3. **WSLg 合成仍是 CPU/远程渲染路径**：应用侧 GL 已由 GPU 完成，但最终呈现经 WSLg
   的 RDP/Weston 通道；极端高分辨率/多传感器场景仍可能受该通道限制。
4. `~/.bashrc` 生效范围限于交互式 shell（见 4.2 注）。
5. Gazebo 启动时的 `Anti-aliasing level ... valid FSAA levels are: [0 4]` 与
   `QML Binding loop detected` 属提示性噪音，不影响运行。

## 14. 风险与后续建议

1. 传感器仿真（LiDAR/相机）负载明显高于 shapes 场景，建议先在带传感器的世界里做一次
   性能抽样；如显存/GPU 利用率吃紧，可考虑改用 Intel Arc 核显（把
   `MESA_D3D12_DEFAULT_ADAPTER_NAME` 换成对应名称）分摊。
2. WSL 内存 15 GiB：当前无内存不足证据，暂不修改 `.wslconfig`；如 Gazebo + Nav2 +
   RViz2 + SLAM 同时运行出现 OOM，再评估 `memory=`。
3. 两个发行版共用同一 WSL2 内核/WSLg：全局操作（`wsl --shutdown`、`wsl --update`）
   会同时影响两者。
4. `rosdep update` 需按需重跑以刷新源索引。

## 15. 环境可用性结论与下一步

基础环境已全部验收通过，可直接用于后续开发。建议顺序：
1. 机器人模型（URDF/xacro）与 `robot_state_publisher`；
2. Gazebo 中加载模型 + 2D/3D LiDAR，`ros_gz_bridge` 桥接 `/scan`、`/tf`；
3. SLAM Toolbox 建图 + RViz2 可视化；
4. Nav2 导航（map_server/AMCL 或 SLAM 定位模式、costmap、planner/controller）；
5. 动态障碍物与避障演示。

本阶段到此为止，未开始任何后续开发。

---

# 附：MuJoCo 与 Booster T1 官方仿真资源验证（第三轮，2026-10-13 晚）

本轮不修改、不卸载既有 ROS 2 / Gazebo / Nav2 / SLAM Toolbox 与工作区（仅追加本报告）。

## 16.1 独立的 MuJoCo Python 环境

| 项目 | 值 |
| --- | --- |
| 环境目录 | `/home/haoyang/venvs/humanoid_mujoco`（`python3 -m venv`，未用 Conda/Docker/Snap/系统 pip） |
| Python | 3.12.3（`/home/haoyang/venvs/humanoid_mujoco/bin/python`） |
| pip | 26.2.1（venv 内） |
| MuJoCo | **3.15.0** |
| NumPy / SciPy / PyYAML / onnxruntime | 2.5.4 / 1.18.1 / 6.0.3 / 1.31.0 |
| PyTorch | **2.14.1+cu130**（经 `booster_deploy/requirements.txt` 安装；`torch.cuda.is_available()` 在 WSL 中返回 True，但**策略按约定用 CPU 运行**） |
| 其它 | evdev 2.0.0、glfw 2.10.2、PyOpenGL 3.1.10、triton 3.8.0、sympy 1.14.0 及 nvidia-* 依赖轮子 |
| 系统包（apt） | `python3-venv`、`python3.12-venv`、`libglfw3`、`git`、`git-lfs`（3.4.1）、`mesa-utils`（已在） |

## 16.2 MuJoCo 基础验证（阶段三）

自建最小模型 `~/mujoco_smoke_test/mj_smoke_model.xml`，无窗口跑 10000 步：

| 指标 | 结果 |
| --- | --- |
| 模型 | nq=8 nv=7 nu=1，timestep 0.002 s |
| 10000 步耗时 | 0.042 s |
| 步/秒 | **237,913 steps/s** |
| 相对实时 | **475.83×** |
| NaN | 无 |
| WSLg viewer | 窗口 `MuJoCo : codex_mujoco_smoke` 打开并正常渲染（截图 `mujoco_viewer.png`） |
| 退出 | 无遗留进程；进程在解释器退出阶段出现一次 segfault（发生在 viewer 正常关闭之后，日志已打印 `viewer closed cleanly`） |

## 16.3 官方仓库与版本（阶段四）

| 仓库 | remote | 分支 | commit SHA | 提交日期 | 工作区状态 |
| --- | --- | --- | --- | --- | --- |
| booster_assets | https://github.com/BoosterRobotics/booster_assets.git | main | `12a97516f57469078707afb587a5f84c1cb110d1` | 2026-09-26 | clean（无本地改动） |
| booster_deploy | https://github.com/BoosterRobotics/booster_deploy.git | main | `7bb1462e2ac742eba5c01659b815e2e99fbe617f` | 2026-09-13 | clean（无本地改动） |

克隆位置：`/home/haoyang/robot_sim_sources/{booster_assets,booster_deploy}`（Ubuntu 24.04 自有 ext4，非 /mnt/c）。
`git lfs ls-files` 在两个仓库均为空 —— 本次涉及的策略文件不是 Git LFS 指针，无需 `git lfs pull`。
**未克隆 booster_gym**（仅作旧实现参考，本阶段不需要）。

## 16.4 Booster Assets 验证（阶段五）

- `python -m pip install -e .` 成功，`import booster_assets` 正常（`booster_assets-1.0.0`）。
- 模型文件：`robots/T1/T1_23dof.xml`、`robots/T1/T1_locomotion.xml` 均存在；`robots/T1/meshes/` 下 STL 齐全；`robots/` 含 K1/T1/T2。
- MuJoCo 直接加载：
  - `T1_23dof.xml` → **nq=30, nv=29, nu=23**（23 自由度），nbody=26，ngeom=48 ✔
  - `T1_locomotion.xml` → nq=19, nv=18, nu=12 ✔
- 加载无缺 mesh、无 XML 报错；**未修改任何官方 XML**。

## 16.5 Booster Deploy 验证（阶段六/七）

- `python -m pip install -r requirements.txt` 成功（torch/numpy/mujoco/scipy/evdev/pyyaml/onnxruntime）。
- `python scripts/deploy.py --list` 实际输出（节选）：

```
Available tasks:
  k1_mj2 : tasks.beyond_mimic.robots.k1.K1MJ2ControllerCfg
  k1_fight : tasks.beyond_mimic.robots.k1.K1FightControllerCfg
  t1_motion_tracking : tasks.beyond_mimic.robots.t1.T1MotionTrackingControllerCfg
  t2_dance : tasks.beyond_mimic.robots.t2.T2DanceControllerCfg
  k1_walk : tasks.locomotion.robots.k1.K1WalkTaskCfg
  t1_walk : tasks.locomotion.robots.t1.T1WalkControllerCfg1      <-- 存在
  t2_walk : tasks.locomotion.robots.t2.T2WalkTaskCfg
```

- `python scripts/deploy.py --help` 支持 `--task/--list/--mujoco/--device/--exit-mode`。
- 任务装配核对（按实际代码，不按 README 猜测）：
  - `tasks/locomotion/robots/t1/__init__.py:14` 使用 `T1_23DOF_CFG.replace(...)`
  - 同文件 `:77` 设置 `self.policy.checkpoint_path = "robots/t1/models/t1_walk.pt"`
  - 同文件 `:80` `register_task("t1_walk", T1WalkControllerCfg1())`
  - 运行时路径解析：`tasks/locomotion/locomotion.py:25-27` → `os.path.join(self.task_path, cfg.checkpoint_path)` = `tasks/locomotion/robots/t1/models/t1_walk.pt`
  - MJCF：`booster_deploy/robots/t1.py` 中 `mjcf_path="{BOOSTER_ASSETS_DIR}/robots/T1/T1_23dof.xml"` → 解析到 booster_assets ✔

> 注意：清单里写的 `tasks/locomotion/models/t1_walk.pt` 在当前官方 main **不存在**；
> 实际路径是 `tasks/locomotion/robots/t1/models/t1_walk.pt`（已按实际目录核对）。

## 16.6 预训练策略验证（阶段八）

Checkpoint 事实：

| 项目 | 值 |
| --- | --- |
| 路径 | `~/robot_sim_sources/booster_deploy/tasks/locomotion/robots/t1/models/t1_walk.pt` |
| 大小 | **2,166,975 bytes** |
| 类型 | Zip archive / TorchScript 归档（首字节 `PK\x03\x04`） |
| Git 管理 | 普通 blob `2d8a16b65fde478775690ada3e64d142c6541530`，**不是 LFS 指针** |
| 加载 | `TorchScriptRunner` 加载成功；CPU 推理；无 shape mismatch、无 joint name mismatch、无缺 mesh、无 NaN |

运行方式：官方运行器 `python scripts/deploy.py --task t1_walk --mujoco --device cpu`；
速度指令经官方 stdin 接口输入（`x y yaw` 三个数，终端提示 `Set command (x, y, yaw):`），
Space 在该运行器中为暂停/继续（README 的 w/s/a/d/q/e 键位属于真机遥控器路径，不是 MuJoCo 运行器）。

实测行为（仿真时间标定；GUI 下仿真约为实时的 0.7～0.85×）：

| 阶段 | 指令 | 观测（仿真时间） | 结论 |
| --- | --- | --- | --- |
| 站立 | (0, 0, 0) | 骨盆高度稳定在 z≈0.684 m，x/y 漂移 <1 cm，yaw 缓慢漂移 ≈+0.2°/s | **稳定站立，不摔** |
| 小幅前进 | vx=0.2 | x 仅 +0.05 m 后停止前进（act_norm 保持 ≈2.6，未进入步态） | 未持续行走 |
| 中幅前进 | vx=0.3 | x 仅 +0.018 m 后停止前进 | 未持续行走 |
| 前进 | vx=0.5 | **9.5 s 仿真时间前进 3.68 m ≈ 0.39 m/s（指令的 77%）**，act_norm 2.7→4.6（步态激活） | **可持续行走** |
| 停止 | (0, 0, 0) | 位置收敛并保持（x 3.765→3.826 m 后稳定），z≈0.683 m，act_norm 回到 ≈2.4 | 停下不摔 |
| 横移 | vy=0.15 | y 位移 ≤1 cm | 无明显横移响应 |
| 旋转 | vyaw=0.3 | yaw 稳定增加，实测 ≈0.16–0.18 rad/s（约指令的 55–60%），两次实验分别旋转 99°/72° 后保持 | **可旋转且保持平衡** |

伴随现象：前进时存在横向漂移（y 约 +0.5 m / 9.5 s 仿真时间）与轻微偏航漂移（≈+4°），
说明当前 main 的 T1 行走策略不严格走直线；低速段（0.2/0.3 m/s）存在“走一步即停”的死区。

## 16.7 booster_gym（旧实现）处理

按约定：**未克隆** booster_gym，未使用 `deploy/models/T1.pt`、`deploy/configs/T1.yaml`、
`resources/T1/T1_locomotion.xml`，未将它们混入新 `t1_walk` 任务；未运行 `train.py` / `play.py` /
`export_model.py`（不训练、不导出）。

## 16.8 本轮安装/创建/修改清单

安装（apt）：`python3-venv`、`python3.12-venv`、`libglfw3`、`git`、`git-lfs`。
安装（venv pip）：mujoco 3.15.0、numpy 2.5.4、scipy 1.18.1、pyyaml 6.0.3、onnxruntime 1.31.0、
torch 2.14.1+cu130（含 nvidia-* / triton / sympy / networkx / jinja2 / filelock / MarkupSafe /
setuptools）、evdev 2.0.0、glfw 2.10.2、PyOpenGL 3.1.10、absl-py、etils、flatbuffers、fsspec、
packaging、protobuf、typing_extensions、websockets、zipp。
创建：`~/venvs/humanoid_mujoco`、`~/robot_sim_sources/{booster_assets,booster_deploy}`、
`~/mujoco_smoke_test/`、`~/t1_walk_evidence/`。
修改：`~/.gitconfig`（`git lfs install` 写入 LFS filter）；本报告文件（追加本节）。
**未修改**：任何 ROS 2 / Gazebo / Nav2 / SLAM Toolbox 配置、工作区内容（除本报告）、
两个官方仓库的任何文件（`git status` 均为 clean）、`/etc/environment`、`.wslconfig`、Windows 设置。

## 16.9 证据文件

`~/t1_walk_evidence/`：

| 文件 | 内容 |
| --- | --- |
| `run3_monitor.log` | vx=0.3 / vy=0.15 / vyaw=0.3 的完整轨迹（含仿真时间） |
| `run4_monitor.log` | vx=0.5 长距离前进 + 旋转 + 动作幅度统计 |
| `run5_monitor.log` | 同一会话内 0.2 → 0.3 → 0.5 的阶跃对比（低速死区证据） |
| `t1_standing.png` | 站立姿态 |
| `t1_after_forward_stop.png` | 前进后停止姿态 |
| `t1_yaw_rotated.png` | 旋转后（朝向约转 180°）姿态 |
| `t1_final.png` | 最终停止姿态 |

## 16.10 未解决问题

1. **低速前进死区**：vx=0.2/0.3 m/s 时策略只迈一步即停（vx=0.5 正常行走约 0.39 m/s）。
   证据：`run5_monitor.log` 中 x 在 0.063/0.083 m 处长时间不变而 act_norm 保持 ≈2.6（未进入步态）。
   影响：无法用“很小”的前进指令起步；如需低速行走，需在官方侧确认（本阶段不修改其配置/参数）。
2. **横向速度无响应**：vy=0.15 m/s 位移 ≤1 cm（`run3_monitor.log`）。未做更大 vy 测试以保持“小幅命令”约定。
3. **行走时横向/偏航漂移**：前进 9.5 s（仿真）横向漂移约 0.5 m、偏航约 +4°。
4. GUI 下仿真速度约为实时的 0.7～0.85×（CPU 推理 + WSLg 渲染开销），非策略问题。
5. MuJoCo viewer 在进程退出阶段偶发 segfault（发生在窗口正常关闭之后），未产生遗留进程。

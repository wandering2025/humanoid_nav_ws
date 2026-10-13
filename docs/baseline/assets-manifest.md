# 资产与策略清单

W01 现场采集；两个外置仓库均 clean，未复制策略或 mesh。机器绑定见 environment.md；原始机器可读证据见 ../evidence/W01/c2-assets.json。

| 仓库 | 实际路径 | origin | 分支 / SHA | dirty | 许可证 |
| --- | --- | --- | --- | --- | --- |
| booster_deploy | /home/haoyang/robot_sim_sources/booster_deploy | https://github.com/BoosterRobotics/booster_deploy.git | main / `7bb1462e2ac742eba5c01659b815e2e99fbe617f` | clean | `/home/haoyang/robot_sim_sources/booster_deploy/LICENSE`（Apache-2.0） |
| booster_assets | /home/haoyang/robot_sim_sources/booster_assets | https://github.com/BoosterRobotics/booster_assets.git | main / `12a97516f57469078707afb587a5f84c1cb110d1` | clean | `/home/haoyang/robot_sim_sources/booster_assets/LICENSE`（BSD-3-Clause） |

| 文件 | 实际路径 | 字节 | SHA256 | Git blob |
| --- | --- | --- | --- | --- |
| checkpoint | /home/haoyang/robot_sim_sources/booster_deploy/tasks/locomotion/robots/t1/models/t1_walk.pt | 2166975 | `ef1d61e19082b83405f4320a08f4cfc2d7d7f003ed3790dab013778ba442dec7` | `2d8a16b65fde478775690ada3e64d142c6541530` |
| mjcf | /home/haoyang/robot_sim_sources/booster_assets/robots/T1/T1_23dof.xml | 18682 | `f00e3410362b1e2267e6db05d25c33d6240bbf2436d62a409f07042bad4f7fb2` | `c0e069b7d72c78baef975549e489b380c3dde5b6` |

运行入口：booster_deploy/scripts/deploy.py 的 t1_walk，策略为 tasks/locomotion/robots/t1/models/t1_walk.pt；配置来源 tasks/locomotion/robots/t1/__init__.py:77 与 booster_deploy/robots/t1.py:134。模型路径由 booster_assets.BOOSTER_ASSETS_DIR 解析。

主 MJCF：T1_23dof.xml；机器人配置名 Booster_T1_23DOF，首个机器人 body 为 trunk。实测 nq=30、nv=29、nu=23、njnt=24（含 free joint）、nbody=26、ngeom=48，timestep=0.002 s。完整关节与 actuator 名称见 c2-assets.json。模型加载成功，无缺失 mesh；未逐一哈希大型 mesh。

robots/T1/meshes/ 为模型 mesh 目录。仓库级许可证已定位，策略与资产是否有额外分发条件留给 W14 核对；本步不授予或宣称分发许可。主 XML 哈希不覆盖其引用资产全部内容，来源固定仓库 SHA。

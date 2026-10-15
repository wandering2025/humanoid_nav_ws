# W11R1V 独立验证流水

2026-10-15T09:33:37.813121+00:00：已完成只读Git及文件冻结核对；开始HEAD 284aba1e39c239063b5ca92725fde951bf31cc72，初始dirty无。113源码/脚本指纹匹配，formal-v1 1371/1371哈希一致。构建/物理运行尚未执行。正在完成宿主审计和预注册；未完成A–G。下一命令：审查并提交verification-plan.yaml及初始审计。无本任务运行PID。

2026-10-15T09:34:26Z：宿主冻结审计PASS，无项目进程；42124/42126空闲，domain34/134未观察到项目参与者，upstream SHA/clean匹配。第一次审计误把自身Python和系统unattended-upgrades计入，读检查退出1；未执行候选，已在host-initial-audit.json保留错误说明。既有evaluate/nav_run --help退出0；smoke无--help且固定旧输出，不调用；cancel入口仅接受路径或--audit-result，绝对路径可绑定新目录。W12旧harness输出硬编码，不直接导入。下一命令：预注册commit，然后阶段A。

2026-10-15T09:36:23.685034+00:00：阶段A完成PASS。build退出0，4包；test退出0；test-result退出0，151tests/0errors/0failures/0skipped。被测HEAD 8cd6adc0af387f2101168c278157fdc87ee8e22d；src/scripts冻结匹配。原XML旁路复制，未删历史结果。协议/适配器45项、真实ROS manager3/规划4/controller2均在有效XML中；模拟/合成输入不冒充物理验证。正在保存检查点，B–G未执行。下一命令：新进程执行冻结startup_probe(1)，重复10次且每次独立Session，首次候选失败即停止。

2026-10-15T09:39:10.725066+00:00：阶段B首次运行后停止，INDEPENDENT_VERIFICATION_FAIL。run 20261015T093701Z_w11r1-independent-v1-startup_01_s0；被测HEAD 61264cfb27d32fb831079d2f85989a8d8eedb7aa；run wrapper退出0、launch退出0；就绪14.219661墙钟秒/10.8sim_s通过，真实FollowPath ACK+预期103结果、两个costmap响应、8节点active、initialpose/clock/scan通过。但冻结startup_probe按就绪事件正常关闭时，navigation_starter在destroy_node中KeyboardInterrupt，PID99620退出-2（日志219–234/300）；其余17子进程exit0，DDS退出0，无残留。冷启动完整通过0/10（仅执行1次，就绪1/1，9次未执行）。环境/障碍/非足触地0，跌倒0，未arm/未发送导航目标。C四例、D完整W11、E取消替换、F完整W12均按首次候选失败停止规则NOT_RUN。原始失败不重跑，未修源码/参数/测试逻辑。下一命令：最终宿主/端口/冻结审计并提交FAIL证据，再更新W11记录/看板/交接。离线首解析器误把signal_handler(SIGINT/SIGTERM)文本计为异常，原件保留；更正派生审计只认真实退出行，结论仍FAIL。

2026-10-15T09:40:52.865192+00:00：最终审计完成。src/scripts指纹与113文件冻结一致；预注册输入无变化、计划字节未改；formal-v1 1371哈希匹配；W11/W12/W13/W11R1历史与docs/results/用户文档无diff。宿主项目进程无、20个曾观察owned PID均不存在，42124/42126空闲，upstreams SHA/clean保持；SIGTERM/SIGKILL0。已完成A、B首例和G；后续NOT_RUN且不重试。最终FAIL。下一命令：审查并提交W11R1V失败证据，再更新W11记录/看板/交接。

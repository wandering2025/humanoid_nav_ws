> 本页保留 `demo-v0.2.1` 固定默认B的发布验收。当前工作分支新增可编辑B点的presentation接口，验证见[presentation/VALIDATION.md](../../presentation/VALIDATION.md)；旧标签与录像未移动或覆盖。

# 优化发布 demo-v0.2.1

**ADVANCED_DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**。本次针对用户反馈增强了动态障碍的可见干扰，减少重复行程和末端绕圈，并验证 RViz 实际规划显示。既有 `demo-v0.2.0` 不移动；新标签只在完整验收和文档提交后创建。

```bash
/home/haoyang/humanoid_nav_ws_advanced/scripts/run_advanced_demo.sh
```

| 项目 | 冻结值 |
|---|---|
| 分支 / worktree | feature/advanced-slam-demo-v0.2 / /home/haoyang/humanoid_nav_ws_advanced |
| 最终实现 SHA | `405ba357689f566a485d945c797e192b4e981162` |
| 导航功能核心 SHA | `2875228d3c27e293c80087453543be0c549f3347`；后续只改显示和GPU启动准入 |
| 完整录像运行源码 SHA | `a19286279bfe0122d2a2ffb918b2ac6ae5d89579`；与最终实现的 scripts/src 一致 |
| 最终证据 SHA | `f62f51324500d5d7dc587fe64cd83c21e0043529` |
| 发布标签 | `demo-v0.2.1`，指向证据和交接齐全后的提交 |
| 完整运行 ID | `20261016T090707Z_advanced_d2063f25-32cf-473b-bd06-ff6f2b578c7e` |
| 最终结果 | SUCCESS；四项接触/跌倒0，所有基础组件与目标工具退出0 |
| 旧标签 | demo-v0.1.0、demo-v0.2.0原对象及目标未移动；没有push |

[最终完整录像](../evidence/advanced-optimization-visual/raw/recording-03/advanced-demo.mp4)：**161.50秒**，2048×852、4fps H264/avc1、无音频。564组真实双窗口抓取，646/646帧完整软件解码，16个关键画面实际检查通过。保持真实单调时钟时间，尾部8秒明确标记运行后证据卡；未加速或合成机器人运动。顺序窗口抓取最大时间差0.263秒、最大采样间隔0.503秒。

SHA256：`9231af1918b061bfc60089f8c48d2b61621b185e28c31eabe9429fbf45c289e9`

录像章节：

- 0:00.00 — A and initial LiDAR map。
- 0:05.75 — West area: live blue plan。
- 0:18.00 — Turn into west gallery。
- 0:31.25 — Northward mapping: map grows。
- 0:44.75 — Turn through north gallery。
- 0:58.00 — East survey around occlusion。
- 1:17.50 — Map ready: cyan A-B overview。
- 1:25.25 — Cart enters actual route。
- 1:32.00 — Cart holds: robot waiting。
- 1:35.75 — Waiting near end of hold。
- 1:39.50 — Cart withdraws。
- 1:45.50 — Route clear: walking resumes。
- 2:02.75 — Final straight approach。
- 2:15.00 — Verified B arrival。
- 2:32.50 — Safe-stop final view。
- 2:36.50 — Post-run verified evidence。

原始视频和帧保持 Git ignored；[最终视频记录](../evidence/advanced-optimization-visual/final-video.json)、[原始哈希索引](../evidence/advanced-optimization-visual/raw-index.json)、[源码与稳定版审计](../evidence/advanced-optimization-visual/source-audit.json)可追溯。完整历史失败均保留，包括本次启动准入拒绝和第二轮视觉概览误标；它们未被作为发布视频。

[验收与限制](optimization-review.md) · [操作](quick-start.md) · [旧demo-v0.2.0发布记录](release-manifest.md)

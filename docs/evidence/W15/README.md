# 当前W15：受控恢复发布通过

**DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**。见[恢复证据索引](recovery/README.md)和[当前发布清单](../../demo/release-manifest.md)。以下原阻塞说明完整保留为历史。

---

# W15 evidence — FINAL_RELEASE_BLOCKED

This directory contains genuine passes and failures; it is not a successful release record.

- `verification.json`: full implementation SHA, four-package build, initial 186 tests, final 55 affected tests, CPU preflight, fail-closed default entry, 99 unchanged baseline files, and clean frozen dependencies.
- `minimal-startup-race.json`: first minimum viewer attempt failed because capture preceded window readiness; original clip/lock preserved.
- `graphics-blocker.json`, `kernel-crash-excerpt.txt`, `minimal-display-02-result.json`: second minimum renderer exit −11, NVIDIA user-mode library fault. No further GPU/GUI/capture load after this fault. Missing thread join in the new checker was fixed offline; the graphical correction is NOT validated. The production simulator already waited for viewer threads.
- `review-summary.json`: preselected S01 SUCCESS; B01 FAIL_TIMEOUT with zero contacts/falls; source_timeout PASS including explicit rearm. These CPU checks do not replace a graphical demonstration or change the W13 36/40 FAILED verdict.
- `fault-fixture-failure.json`: first fault-check attempt failed before simulation because its relocated launch fixture was absent; corrected attempt is separately retained.
- `diagnostic-video-validation.json`: both failed diagnostic clips fully decode (317/317 and 536/536 frames). Neither is a final presentation video. No successful full navigation video exists.
- `final-process-port-audit.json`: no owned/project/review/capture processes; ports42127–42132 bindable; no GPU query in this audit.
- `raw-index.json`: SHA256/size for 155 new raw logs, clips, snapshots and physical traces. Raw files stay ignored and local; no old W13/W14 evidence changed.

Source-timeout wrapper uses the unchanged W10 test/predicates and copies its fixtures into the unique W15 raw directory. S01/B01 reuse frozen W13 cases and evaluator, without rewriting any W13 campaign. The legacy CPU harness does not persist the exact DDS exit code, so its results are not used to authorize the DDS teardown exception.

The default command now returns3 before GPU access. The committed GPU profile and the local halt both block GUI startup. No release tag was created. No W16 or automatic continuation is scheduled.

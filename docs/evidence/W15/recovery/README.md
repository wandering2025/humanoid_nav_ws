# W15 controlled recovery evidence

Final state: **DEMO_RELEASE_READY_WITH_KNOWN_LIMITATIONS**. Implementation `21df4b116f0e987690c6288e400c7fa0e45a65b5`, final evidence `945eb32bd774a206743ab6d06432c6024bb0504a`, recording source `127a1e7672f72616de728c84f41701669a41f1c9`. Release tag `demo-v0.1.0` points to the final handoff commit.

- [Recovery authorization and original halt](recovery-start.json), [original halt bytes](prior-gpu-halt.json), [minimum-stage authorization](minimal-authorization.json).
- [Offline checks](offline-checks.json): four packages built;186 tests;99 frozen nonvisual files unchanged. Setup/import failures remain in raw logs; no dependency install.
- [B01 source comparison](b01-source-audit.json), [single B01 control](b01-control-summary.json): SUCCESS; prior timeout not reproduced; no reliability-rate claim.
- [Minimum production GUI](minimal-summary.json): SUCCESS_STARTUP_CHECK; original close/join; visual disk-size error preserved then corrected without another minimal GUI run.
- [Full rehearsal](full-summary.json): SUCCESS, dynamicPASS, all exits0.
- [Recording attempt01](recording-attempt-01.json): demoSUCCESS_WITH_WARNING under unchanged DDS gates, captureFAIL with0frames before readiness. Raw failure untouched.
- [Final recording run](final-summary.json): SUCCESS, dynamicPASS, no contacts/fall, safe stop, all exits0, no remaining processes.
- [Video review](video-review.json): 617/617 decoded, selected encoded frames visually reviewed; real-time captured sequence and explicitly labeled result card.
- [Publication equivalence](publication-equivalence.json): only help/profile informational status changes after recorded SHA.
- [Final process/history audit](final-audit.json): no tasks remain, seven ports bindable, original failure branch and evidence preserved, frozen upstreams clean.
- [699-file raw index](raw-index.json): original traces, PNG frames, logs, kernel deltas, movie sizes/SHA256. Raw files stay ignored.

Video: [demo.mp4](../raw/recovery/final-recording-02/demo.mp4),1896620bytes, SHA256 `9980c060692af05c26f6ed83ac2a3a5b5dd88935a4cc72de8fdd1966470e0df0`. Full path relative to workspace: `docs/evidence/W15/raw/recovery/final-recording-02/demo.mp4`.
Prior W15 evidence/index is preserved under the parent directory. This recovery did not erase the original blocked decision or W13 FAILED36/40.

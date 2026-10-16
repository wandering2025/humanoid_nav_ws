# Optimized graphical delivery evidence

Final run `20261016T090707Z_advanced_d2063f25-32cf-473b-bd06-ff6f2b578c7e` at `a19286279bfe0122d2a2ffb918b2ac6ae5d89579`: SUCCESS. Final implementation `405ba357689f566a485d945c797e192b4e981162`; navigation core2875228 unchanged.

- [Final run](final-run.json): actual16.10s wait,7.506m resumed travel,29.898m total sampled route, contact/fall0, verified B, all exits0/cleanup/port.
- [Video](final-video.json):161.5s,646 full decoded frames,564 real paired captures,16 keyframes reviewed. [Playable file](raw/recording-03/advanced-demo.mp4).
- [Map coverage](final-map-coverage.json): actual SLAM PGM; west/north100%, east95.57%, real cart shadow retained.
- [Path correspondence](live-path-correspondence.json):23 geometries/frame match actual planner,11 byte matches, overview never blue. [Original strict audit](live-path-topics.json) remains FAILED on all-byte equality; CPU reproduction shows unequal CDR fill bytes for equal decoded messages. No missing-message or all-header-equality claim.
- [Source/cleanup audit](source-audit.json), [raw index](raw-index.json), [all attempts](attempts.json).1177 old artifacts rechecked;1489 new/raw artifacts plus1128 frame images bound/verified. Old failed evidence, old release video and original GPU latch preserved.
- [Minimal GUI](minimal-01.json), [full rehearsal](rehearsal-01.json), [startup-only recording01 failure](recording-01-startup-failure.json), [recording02 visual failure](recording-02-visual-failure.json). Recording02 runtime succeeded but overview briefly blue;579 raw pairs retained, no release encode.
- [18 focused CPU checks](final-focused-checks.json). Existing full194PASS/1 historical source assertion excluded, build, control/intervention and moving-stop evidence remain in [previous optimization evidence](../advanced-optimization/README.md).

GPU runtime thresholds and robot safety unchanged. Advanced-only admission allows5s≤60% GPU load (default40%) after desktop bursts were measured; no drivers/display/dependencies modified. Analysis scripts are in tools/, executed logs/recording remain ignored in raw/. See [current release manifest](../../advanced/optimization-release-manifest.md) for exact source/evidence/tag and known limitations.

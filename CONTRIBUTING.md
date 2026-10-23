# Development

Use the [setup guide](docs/setup.md), then run `./scripts/test.sh` before submitting changes. The suite covers simulation contracts, gait-command mapping, scene events, LiDAR, exploration, safety shutdown, and evaluation predicates.

Keep physics, policy, sensor, and ROS time consistent. Route all navigation requests through the command gate and final collision filtering. Simulation ground truth belongs in evaluation; scene geometry and obstacle trajectories stay outside the exploration and planning inputs.

Changes to calibrated control files require validation and an intentional update to `scripts/advanced/runtime-lock.json`. The manifest makes these checks independent of historical Git tags and supports fresh source snapshots.

For a behavioral change, record the task/configuration, source revision, run identifier, physical outcome, and component exits. Keep raw run data outside Git and commit a compact result summary when it supports a documented claim.

## Public source snapshot

`.gitattributes` defines the publication boundary: source packages, runtime helpers, current tests, English documentation, and selected validation artifacts. Planning records, machine diagnostics, earlier reports, and raw captures remain in the development repository and its history.

Create a compact snapshot from a committed revision:

```bash
mkdir -p dist
git archive --format=tar.gz --prefix=humanoid-nav/ HEAD > dist/humanoid-nav.tar.gz
```

Extract into a fresh directory and initialize Git there to publish it as an independent personal project. Keep `dependencies.repos`, the runtime contract manifest, and the selected validation records with the source.

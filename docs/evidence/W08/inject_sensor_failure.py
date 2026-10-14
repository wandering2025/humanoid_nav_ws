"""Test-only sensor exception at t=4s; not a product parameter or W10 suite."""
from t1_mujoco.lidar import RayLidar
original = RayLidar.sample

def injected(self, snapshot):
    if snapshot.stamp_ns >= 4_000_000_000:
        raise RuntimeError('W08 injected raycast failure')
    return original(self, snapshot)

RayLidar.sample = injected
from t1_mujoco.simulation import main
raise SystemExit(main())

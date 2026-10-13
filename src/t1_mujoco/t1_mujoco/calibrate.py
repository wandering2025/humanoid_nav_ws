"""W05 calibration suites: python -m t1_mujoco.calibrate <suite> [--out DIR].

Raw per-step traces go to the ignored run directory; the summary CSV and
manifest are small and meant to be copied into docs/evidence/W05.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

from .calibration import Segment, TrialSpec, run_trial, summarize, TRACE_FIELDS

HEADINGS = (0.0, 90.0, 180.0)  # repetitions: same robot-frame initial state
ZERO = (0.0, 0.0, 0.0)


def protocol(command, hold=10.0):
    """1 s zero (READY baseline) -> command -> 6 s zero; warmup precedes it."""
    return [Segment(1.0, ZERO), Segment(hold, command), Segment(6.0, ZERO)]


def c1_families():
    """(family, [command levels in escalation order]). Policy units."""
    fam = [
        ('zero', [ZERO]),
        ('vx', [(v, 0.0, 0.0) for v in (0.2, 0.3, 0.4, 0.5)]),
        ('wz+', [(0.0, 0.0, w) for w in (0.15, 0.3)]),
        ('wz-', [(0.0, 0.0, -w) for w in (0.15, 0.3)]),
        ('vx_wz+', [(0.5, 0.0, 0.15), (0.5, 0.0, 0.3)]),
        ('vx_wz-', [(0.5, 0.0, -0.15), (0.5, 0.0, -0.3)]),
        # Beyond the previously observed range: escalate stepwise.
        ('vx_hi', [(v, 0.0, 0.0) for v in (0.6, 0.7, 0.8, 0.9, 1.0)]),
        ('wz_hi+', [(0.0, 0.0, w) for w in (0.5, 0.75, 1.0)]),
        ('wz_hi-', [(0.0, 0.0, -w) for w in (0.5, 0.75, 1.0)]),
        ('vx_wz_hi+', [(0.7, 0.0, 0.3), (0.5, 0.0, 0.5), (0.8, 0.0, 0.5)]),
        ('vx_wz_hi-', [(0.7, 0.0, -0.3), (0.5, 0.0, -0.5), (0.8, 0.0, -0.5)]),
        ('reverse', [(-v, 0.0, 0.0) for v in (0.2, 0.3, 0.5)]),
        ('lateral+', [(0.0, v, 0.0) for v in (0.15, 0.3, 0.5)]),
        ('lateral-', [(0.0, -v, 0.0) for v in (0.15, 0.3, 0.5)]),
    ]
    return [(name, [(c, protocol(c)) for c in levels]) for name, levels in fam]


def c2_families():
    """Dead-zone, hysteresis and ramp experiments (policy units)."""
    fam = [('vx_sweep', [((v, 0.0, 0.0), protocol((v, 0.0, 0.0)))
                         for v in [round(0.05 * i, 3) for i in range(1, 8)] +
                         [0.375, 0.4, 0.425, 0.45, 0.475]]),
           ('wz_sweep', [((0.0, 0.0, w), protocol((0.0, 0.0, w)))
                         for w in (0.025, 0.05, 0.075, 0.1, 0.125, 0.2, 0.25)])]
    # Hysteresis: start at u=0.5, then step down and hold.
    hyst = []
    for low in (0.4, 0.3, 0.2, 0.15, 0.1):
        hyst.append(((low, 0.0, 0.0), [Segment(1.0, ZERO), Segment(4.0, (0.5, 0.0, 0.0)),
                                       Segment(10.0, (low, 0.0, 0.0)), Segment(6.0, ZERO)]))
    fam.append(('hysteresis', hyst))
    # Slow ramp 0 -> 0.5 over 10 s (25 steps of 0.4 s), then hold.
    ramp = [Segment(1.0, ZERO)] + [Segment(0.4, (0.02 * (i + 1), 0.0, 0.0)) for i in range(25)] + \
        [Segment(4.0, (0.5, 0.0, 0.0)), Segment(6.0, ZERO)]
    fam.append(('ramp', [((0.5, 0.0, 0.0), ramp)]))
    # Combined forward + small yaw near the forward threshold.
    fam.append(('vx_wz_low', [((v, 0.0, w), protocol((v, 0.0, w)))
                              for v, w in ((0.4, 0.15), (0.4, -0.15), (0.4, 0.3), (0.4, -0.3), (0.45, 0.3))]))
    # Hypothesis: gait is gated on the command-vector norm, not per axis.
    fam.append(('norm_gate', [((v, y, w), protocol((v, y, w)))
                              for v, y, w in ((0.3, 0.0, 0.3), (0.3, 0.0, -0.3), (0.2, 0.0, 0.3),
                                              (0.25, 0.25, 0.0), (0.3, 0.3, 0.0), (0.0, 0.3, 0.3))]))
    # Yaw-only then add forward: does turning in place help start walking?
    fam.append(('turn_then_walk', [((0.3, 0.0, 0.3), [Segment(1.0, ZERO), Segment(3.0, (0.0, 0.0, 0.3)),
                                                      Segment(8.0, (0.3, 0.0, 0.3)), Segment(6.0, ZERO)])]))
    return fam


def c3_families():
    """Envelope design data: combined grid, in-place turns, stop phase, warmup."""
    fam = []
    for ux in (0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0):
        fam.append((f'grid_ux{ux:g}', [((ux, 0.0, w), protocol((ux, 0.0, w)))
                                       for w in (0.0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4, 0.5, -0.5)]))
    fam.append(('inplace', [((0.0, 0.0, s * w), protocol((0.0, 0.0, s * w)))
                            for w in (0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.7, 0.8, 0.9, 1.0) for s in (1, -1)]))
    # Zero command issued at 12 different points of the gait cycle.
    for name, cmd in (('stop_vx0.4', (0.4, 0.0, 0.0)), ('stop_vx0.6', (0.6, 0.0, 0.0)),
                      ('stop_vx0.8', (0.8, 0.0, 0.0)), ('stop_vx1.0', (1.0, 0.0, 0.0)),
                      ('stop_turn+', (0.6, 0.0, 0.5)), ('stop_turn-', (0.6, 0.0, -0.5)),
                      ('stop_inplace+', (0.0, 0.0, 1.0)), ('stop_inplace-', (0.0, 0.0, -1.0))):
        fam.append((name, [((cmd[0], cmd[1], cmd[2]), protocol(cmd, hold=round(8.0 + 0.06 * j, 2)))
                           for j in range(12)]))
    return fam


def warmup_trials():
    """Command u=(0.6,0,0) immediately at READY after shorter warmups."""
    return [('warmup', [((0.6, 0.0, 0.0), [Segment(8.0, (0.6, 0.0, 0.0)), Segment(6.0, ZERO)], w)
                        for w in (0.5, 1.0, 1.5, 2.0)])]


SUITES = {'c1': c1_families, 'c2': c2_families, 'c3': c3_families, 'warmup': warmup_trials}


def git(*args):
    from .recording import git as run_git
    return run_git(os.environ.get('T1_WS', Path.cwd()), *args)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', choices=sorted(SUITES))
    parser.add_argument('--out', default='')
    parser.add_argument('--headings', default=','.join(str(h) for h in HEADINGS))
    parser.add_argument('--no-escalation-stop', action='store_true')
    args = parser.parse_args(argv)
    import torch
    import mujoco
    from .runtime import load_task_config, inspect_runtime
    from .recording import sha256
    torch.set_num_threads(1)
    upstream = os.environ['T1_UPSTREAM']
    runtime = inspect_runtime(upstream, os.environ.get('T1_ASSETS', ''), 'sim_only', 't1_walk')
    cfg = load_task_config(upstream)
    ws = Path(os.environ.get('T1_WS', Path.cwd()))
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = Path(args.out) if args.out else ws / 'run_data' / f'{stamp}_dev_w05_{args.suite}'
    out.mkdir(parents=True, exist_ok=False)
    headings = [float(h) for h in args.headings.split(',')]
    dirty = git('status', '--short')
    manifest = dict(schema=1, suite=args.suite, created_utc=datetime.now(timezone.utc).isoformat(),
                    code_sha=git('rev-parse', 'HEAD'), dirty=bool(dirty), dirty_files=dirty.splitlines(),
                    upstream_sha={'booster_deploy': git_at(upstream), 'booster_assets': git_at(runtime['assets_path'])},
                    checkpoint_sha256=sha256(runtime['checkpoint_path']), mjcf_sha256=sha256(runtime['mjcf_path']),
                    python=sys.executable, mujoco=mujoco.__version__, torch=torch.__version__,
                    command_line=[sys.executable, '-m', 't1_mujoco.calibrate', *sys.argv[1:]],
                    protocol='warmup 2.0 s (STARTING, forced zero) then segments; commands are policy units '
                             'fed through InputBuffer each policy step; headings rotate only the initial world yaw',
                    headings_deg=headings, policy_dt=cfg.policy_dt, decimation=cfg.mujoco.decimation,
                    physics_dt=cfg.mujoco.physics_dt, trials=[])
    started_wall = time.monotonic()
    trace_file = (out / 'trace.csv').open('w', newline='')
    trace = csv.DictWriter(trace_file, fieldnames=TRACE_FIELDS + ('t_cmd',), extrasaction='ignore')
    trace.writeheader()
    summaries = []
    for family, levels in SUITES[args.suite]():
        stopped = None
        commands = [level[0] for level in levels]
        for j, level in enumerate(levels):
            command, segments = level[0], level[1]
            warmup = level[2] if len(level) > 2 else 2.0
            name = f'{family}:{command[0]:+.3f},{command[1]:+.3f},{command[2]:+.3f}'
            if len(level) > 2:
                name += f'@w{warmup:g}'
            elif commands.count(command) > 1:
                name += f'#{j}'
            if stopped and not args.no_escalation_stop:
                manifest['trials'].append(dict(name=name, skipped=f'escalation stopped after {stopped}'))
                continue
            for rep, heading in enumerate(headings):
                spec = TrialSpec(name, segments, heading_deg=heading, warmup=warmup, rep=rep, tags=dict(family=family))
                wall0 = time.monotonic()
                result = run_trial(cfg, spec, record=trace.writerow)
                wall = time.monotonic() - wall0
                rows = summarize(result)
                for item in rows:
                    item.update(family=family, trial_state=result['final_state'], fault=result['fault'])
                summaries.extend(rows)
                manifest['trials'].append(dict(name=name, rep=rep, heading_deg=heading, final_state=result['final_state'],
                                               fault=result['fault'], sim_end=result['sim_end'], wall_s=wall))
                print(f'{name} rep{rep} h{heading:g}: {result["final_state"]} sim={result["sim_end"]:.2f}s wall={wall:.2f}s',
                      flush=True)
                if result['final_state'] != 'READY':
                    stopped = name
    trace_file.close()
    keys = []
    for item in summaries:
        keys.extend(k for k in item if k not in keys)
    with (out / 'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for item in summaries:
            writer.writerow({k: (round(v, 5) if isinstance(v, float) and math.isfinite(v) else v)
                             for k, v in item.items()})
    manifest.update(ended_utc=datetime.now(timezone.utc).isoformat(), wall_duration_s=time.monotonic() - started_wall,
                    sim_time_total_s=sum(t.get('sim_end', 0.0) for t in manifest['trials']),
                    summary_sha256=sha256(out / 'summary.csv'))
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('W05_RUN ' + str(out), flush=True)
    return 0


def git_at(path):
    from .recording import git as run_git
    return run_git(path, 'rev-parse', 'HEAD')


if __name__ == '__main__':
    sys.exit(main())

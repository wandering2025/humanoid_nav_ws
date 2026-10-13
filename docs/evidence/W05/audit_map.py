"""Audit the COMMITTED command_map.yaml against every W05 suite trace (read-only).

Same statistic as build_command_map.py (largest 2 s window mean, >=3 s after
onset, constant-command segments >=8 s). Reports violations (measured faster
than M^-1(u)), the slowest ratio measured/expected and the minimum walking
speed inside the envelope. usage: python3 audit_map.py <run dir>... > json
"""
import importlib.util
import json
import math
from pathlib import Path
import sys

ws = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ws / 'src/t1_mujoco'))
from t1_mujoco.command_map import CommandMap  # noqa: E402

spec = importlib.util.spec_from_file_location('b', Path(__file__).with_name('build_command_map.py'))
src = Path(__file__).with_name('build_command_map.py').read_text()
# Reuse window_stats only (the builder's top level would rewrite the map).
ns = {}
exec(src[src.index('def window_stats'):src.index('groups = [g for run')], {'math': math, 'csv': __import__('csv'),
     'DT': 0.02, 'SKIP_S': 3.0, 'WIN_S': 2.0, 'MIN_SEG_S': 8.0}, ns)
window_stats = ns['window_stats']

m = CommandMap.load(ws / 'src/t1_mujoco/config/command_map.yaml')
env = m.env
u_x_max = m.tables['vx'].forward(env['vx_max'])
checks, violations, slow, mins = 0, [], [], []
for run in map(Path, sys.argv[1:]):
    for g in window_stats(run / 'trace.csv'):
        ux, uw = g['ux'], g['uwz']
        if g['uy'] != 0:
            continue
        if ux > 0:
            if not (0.4 - 1e-9 <= ux <= u_x_max + 1e-9):
                continue
            t = m.tables['wz_walking_pos' if uw > 0 else 'wz_walking_neg']
            if abs(uw) > t.forward(env['wz_walking_max']) + 1e-9:
                continue
            e = m.expected((ux, 0.0, uw))
            pairs = [('vx', g['vx_wmax'], e[0])] + ([('wz', g['wz_wmax'], e[2])] if uw else [])
            mins.append(dict(run=run.name, trial=g['trial'], rep=g['rep'], u=[ux, uw], vx_wmax=g['vx_wmax'], expected_vx=e[0]))
        elif ux == 0 and uw:
            side = 'pos' if uw > 0 else 'neg'
            lo, hi = (m.tables['wz_in_place_' + side].forward(env[k]) for k in ('wz_in_place_min', 'wz_in_place_max'))
            if not (lo - 1e-9 <= abs(uw) <= hi + 1e-9):
                continue
            pairs = [('wz_in_place', g['wz_wmax'], m.expected((0.0, 0.0, uw))[2])]
        else:
            continue
        for axis, meas, e in pairs:
            checks += 1
            slow.append((meas / abs(e), axis, run.name, g['trial'], g['rep']))
            if meas > abs(e) + 1e-9:
                violations.append(dict(run=run.name, trial=g['trial'], rep=g['rep'], axis=axis, measured=meas, expected=e))
slow.sort()
print(json.dumps(dict(runs=sys.argv[1:], checks=checks, violations=violations,
                      slowest_ratio_measured_over_expected=[dict(ratio=r, axis=a, run=n, trial=t, rep=p) for r, a, n, t, p in slow[:6]],
                      min_walking_window_max_vx=sorted(mins, key=lambda x: x['vx_wmax'])[:4]), indent=1))

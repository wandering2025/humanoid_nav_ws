"""Build src/t1_mujoco/config/command_map.yaml from the W05 C3 summary.

Each table row is (v_phys, u_pol) where v_phys is the LARGEST 2 s window mean
(>=3 s after onset, raw traces) observed for that u_pol over all headings,
contexts and partner-axis values, rounded UP to 0.005: the robot is expected not to exceed a request.
Envelope limits below are design choices; their reasons are inline and in
locomotion-envelope.md. Exits 1 if any measured point inside the envelope is
faster than the map predicts. usage: build_command_map.py <run dir>... <tested sha>
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

runs, tested_sha = [Path(x) for x in sys.argv[1:-1]], sys.argv[-1]
ws = Path(__file__).resolve().parents[3]
up = lambda x: math.ceil(round(x / 0.005, 6)) * 0.005
DT, SKIP_S, WIN_S, MIN_SEG_S = 0.02, 3.0, 2.0, 8.0


def window_stats(trace_path):
    """Per (trial, rep, segment) with constant command >= 8 s: the largest
    2 s window mean of forward speed and |yaw rate|, from 3 s after onset."""
    out, group, key = [], [], None

    def flush():
        if len(group) * DT + 1e-9 < MIN_SEG_S:
            return
        g = group[int(round(SKIP_S / DT)):]
        n = int(round(WIN_S / DT))
        vx = [float(r['v_meas_x']) for r in g]
        yaw = [float(group[0]['yaw_w'])]
        for r in group[1:]:
            yaw.append(yaw[-1] + math.remainder(float(r['yaw_w']) - yaw[-1], math.tau))
        yaw = yaw[int(round(SKIP_S / DT)):]
        if len(g) < n + 1:
            return
        csum = [0.0]
        for v in vx:
            csum.append(csum[-1] + v)
        vmax = max(abs(csum[i + n] - csum[i]) / n for i in range(len(vx) - n + 1))
        wmax = max(abs(yaw[i + n] - yaw[i]) / (n * DT) for i in range(len(yaw) - n))
        r0 = group[0]
        out.append(dict(trial=r0['trial'], rep=r0['rep'], segment=r0['segment'], family=r0['trial'].split(':')[0],
                        ux=float(r0['u_pol_x']), uy=float(r0['u_pol_y']), uwz=float(r0['u_pol_wz']),
                        vx_wmax=vmax, wz_wmax=wmax, seconds=len(group) * DT))

    with trace_path.open() as f:
        for r in csv.DictReader(f):
            k = (r['trial'], r['rep'], r['segment'])
            if k != key:
                if group:
                    flush()
                group, key = [], k
            group.append(r)
    if group:
        flush()
    return out


groups = [g for run in runs for g in window_stats(run / 'trace.csv')]
near = lambda a, b: abs(a - b) < 1e-9
U_WALK = (0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)  # levels with combined-yaw data; 0.4 = walking floor (0.35 fails)
U_TURN = tuple(round(0.05 * i, 3) for i in range(1, 11))   # walking yaw (dense 0.05 steps)
U_INPLACE = tuple(round(0.6 + 0.025 * i, 3) for i in range(17))  # below 0.6: up to 22.5 deg stop rebound
walking = [g for g in groups if g['ux'] >= 0.4 - 1e-9 and g['uy'] == 0]
inplace = [g for g in groups if g['ux'] == 0 and g['uy'] == 0 and abs(g['uwz']) >= 0.55 - 1e-9]


def conservative(levels, select, key):
    """Max window mean per level, rounded up, then made strictly increasing."""
    table = []
    for u in levels:
        values = [g[key] for g in select(u)]
        assert values, ('no data', key, u)
        v = round(up(max(values)), 3)
        if table and v <= table[-1][0]:
            v = round(table[-1][0] + 0.005, 3)
        table.append([v, u])
    return table


vx = conservative(U_WALK, lambda u: [g for g in walking if near(g['ux'], u) and abs(g['uwz']) <= 0.5], 'vx_wmax')
wz_walk, wz_inplace = {}, {}
for side, sign in (('pos', 1), ('neg', -1)):
    wz_walk[side] = conservative(U_TURN, lambda u: [g for g in walking if near(g['uwz'], sign * u)], 'wz_wmax')
    wz_inplace[side] = conservative(U_INPLACE, lambda u: [g for g in inplace if near(g['uwz'], sign * u)], 'wz_wmax')
envelope = dict(
    # Floor = conservative speed at u=0.4. Requests below are ZEROED (robot stands).
    vx_min=vx[0][0],
    # Worst stop distance at u=0.8 (<=0.772 m/s incl. turning) is 0.353 m; one verified
    # table row remains above the limit, so no edge extrapolation.
    vx_max=0.75,
    # Walking yaw verified to |u_wz|=0.5 at every u_x in [0.4, 1.0].
    wz_walking_max=0.45,
    wz_in_place_min=round(up(max(wz_inplace['pos'][0][0], wz_inplace['neg'][0][0])), 3),
    wz_in_place_max=0.85,
)
data = dict(
    schema=1, calibrated=True,
    evidence=dict(report='docs/evidence/W05/locomotion-envelope.md',
                  runs={run.name: hashlib.sha256((run / 'summary.csv').read_bytes()).hexdigest() for run in runs},
                  statistic='largest 2 s window mean, >=3 s after command onset, segments >=8 s (raw traces)',
                  tested_code_sha=tested_sha, builder='docs/evidence/W05/build_command_map.py',
                  validation='docs/evidence/W05/command-map-validation.json'),
    applicability=('Frozen booster_deploy 7bb1462e t1_walk policy, booster_assets 12a97516 T1_23dof, MuJoCo 3.15.0, '
                   'flat z=0 ground of the original scene, policy_dt 0.02/decimation 10, warmup 2.0 s. '
                   'Headless deterministic simulation; not a hardware or safety certification. '
                   'Re-run W05 suites after changing scene friction/terrain, model, policy or timing.'),
    units='v_des in base_footprint: vx m/s, vy m/s, wz rad/s; u_pol policy units (upstream bound 1.0)',
    capabilities=dict(forward=True, reverse_allowed=False, lateral_allowed=False, in_place_turn_allowed=True),
    envelope=envelope,
    deadband=('0 < vx < vx_min -> vx=0; with vx=0: |wz| < wz_in_place_min -> wz=0. Never raised to a gait '
              'command. While walking, |wz| below the first table row maps proportionally to zero.'),
    tables=dict(vx=vx, wz_walking_pos=wz_walk['pos'], wz_walking_neg=wz_walk['neg'],
                wz_in_place_pos=wz_inplace['pos'], wz_in_place_neg=wz_inplace['neg']),
)
header = ('# W05 calibrated v_des -> u_pol contract (interfaces.md §6.2). GENERATED by\n'
          '# docs/evidence/W05/build_command_map.py from measured data; do not hand-edit values.\n'
          '# Rows: [v_phys conservative max observed, u_pol]. See docs/evidence/W05/locomotion-envelope.md.\n')
import yaml  # noqa: E402
out = ws / 'src/t1_mujoco/config/command_map.yaml'
out.write_text(header + yaml.safe_dump(data, sort_keys=False, width=110))

# Validate: every measured point inside the admitted envelope must not exceed
# the map's expected physical value for its u_pol (interpolation conservative).
sys.path.insert(0, str(ws / 'src/t1_mujoco'))
from t1_mujoco.command_map import CommandMap  # noqa: E402
m = CommandMap.load(out)
checks, violations = 0, []
u_x_max = m.tables['vx'].forward(envelope['vx_max'])
for g in walking:
    ux, uw = g['ux'], g['uwz']
    if ux > u_x_max + 1e-9:
        continue
    if abs(uw) > m.tables['wz_walking_pos' if uw > 0 else 'wz_walking_neg'].forward(envelope['wz_walking_max']) + 1e-9:
        continue
    exp = m.expected((ux, 0.0, uw))
    for axis, meas, e in (('vx', g['vx_wmax'], exp[0]), ('wz', g['wz_wmax'], exp[2])):
        if axis == 'wz' and uw == 0:
            continue
        checks += 1
        if meas > abs(e) + 1e-9:
            violations.append(dict(trial=g['trial'], rep=g['rep'], axis=axis, u=[ux, uw], measured=meas, expected=e))
for g in inplace:
    uw = g['uwz']
    side = 'pos' if uw > 0 else 'neg'
    lo, hi = (m.tables['wz_in_place_' + side].forward(envelope[k]) for k in ('wz_in_place_min', 'wz_in_place_max'))
    if not (lo - 1e-9 <= abs(uw) <= hi + 1e-9):
        continue
    checks += 1
    e = m.expected((0.0, 0.0, uw))[2]
    if g['wz_wmax'] > abs(e) + 1e-9:
        violations.append(dict(trial=g['trial'], rep=g['rep'], axis='wz_in_place', u=[0.0, uw],
                               measured=g['wz_wmax'], expected=e))
report = dict(map_sha256=hashlib.sha256(out.read_bytes()).hexdigest(), checks=checks, violations=violations,
              statistic='largest 2 s window mean >=3 s after onset', groups=len(groups),
              note='for every constant-command segment whose u lies in the admitted envelope: measured <= M^-1(u)')
(ws / 'docs/evidence/W05/command-map-validation.json').write_text(json.dumps(report, indent=1) + '\n')
print(json.dumps(dict(envelope=envelope, tables=data['tables'], checks=checks, violations=len(violations)), indent=1))
sys.exit(1 if violations else 0)

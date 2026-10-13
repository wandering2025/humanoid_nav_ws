"""Stop envelope over every W05 suite: stop segment after a constant command.
usage: python3 stops_all.py summary.csv... """
import csv, collections, sys
groups = collections.defaultdict(list)
for path in sys.argv[1:]:
    rows = list(csv.DictReader(open(path)))
    by = {(r['trial'], r['rep'], r['segment']): r for r in rows}
    for r in rows:
        if not r.get('stop_time_s'):
            continue
        prev = by[(r['trial'], r['rep'], str(int(r['segment']) - 1))]
        ux, uwz = float(prev['ux']), float(prev['uwz'])
        if float(prev['uy']) != 0 or ux < 0:
            continue
        if ux == 0:
            if abs(uwz) < 0.6 - 1e-9:
                key = 'in-place |u|<0.6 (excluded from envelope)'
            else:
                key = f"in-place |u| {'0.60-0.85' if abs(uwz) <= 0.85 + 1e-9 else '>0.85'} {'+' if uwz > 0 else '-'}"
        elif ux < 0.4:
            continue
        else:
            key = f'walk u_x {ux:.2f}' if ux in (0.4, 0.5, 0.6, 0.7, 0.8) else f'walk u_x {"<=0.8" if ux <= 0.8 else ">0.8"} (other)'
        groups[key].append((r, prev, path.split('/')[-1]))
for k in sorted(groups):
    v = groups[k]
    f = lambda key: [float(r[key]) for r, _, _ in v]
    held = all(r['settled_hold'] == 'True' for r, _, _ in v)
    print(f"{k:42s} n={len(v):4d} v<={max(abs(float(p['steady_vx'])) for _, p, _ in v):.3f} "
          f"dmax={max(f('stop_dist_max_m')):.3f} tmax={max(f('stop_time_s')):.2f} "
          f"yaw=[{min(f('stop_yaw_deg')):.1f},{max(f('stop_yaw_deg')):.1f}] held={held}")

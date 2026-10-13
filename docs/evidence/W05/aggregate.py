"""Condense a W05 summary.csv into a markdown table (min..max over repetitions).

usage: python aggregate.py summary.csv [--segment N] > table.md
Command segment is 1 and stop segment 2 for the standard protocol.
"""
import csv
import math
import sys


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def span(rows, key, digits=3):
    values = [num(r.get(key)) for r in rows]
    values = [v for v in values if math.isfinite(v)]
    if not values:
        return '—'
    lo, hi = min(values), max(values)
    if hi - lo < 10 ** -digits:
        return f'{lo:.{digits}f}'
    return f'{lo:.{digits}f}..{hi:.{digits}f}'


def main(path, command_segment='1', stop_segment='2'):
    rows = list(csv.DictReader(open(path)))
    names = []
    for r in rows:
        if r['trial'] not in names:
            names.append(r['trial'])
    print('| trial (u_x,u_y,u_wz) | n | vx m/s | vy m/s | wz rad/s | start 0.05 m/rad s | t90 s | lateral m | max yaw dev ° '
          '| max tilt ° | min z m | single-support | non-foot ground | stop time s | stop dist m | stop yaw ° | final |')
    print('|' + ' --- |' * 17)
    for name in names:
        cmd = [r for r in rows if r['trial'] == name and r['segment'] == command_segment]
        stop = [r for r in rows if r['trial'] == name and r['segment'] == stop_segment]
        final = sorted({r['trial_state'] for r in rows if r['trial'] == name})
        print(f"| {name} | {len(cmd)} | {span(cmd, 'steady_vx')} | {span(cmd, 'steady_vy')} | {span(cmd, 'steady_wz')} "
              f"| {span(cmd, 'onset_move_s', 2)} | {span(cmd, 'onset_t90_s', 2)} | {span(cmd, 'disp_lat', 2)} "
              f"| {span(cmd, 'max_abs_yaw_dev_deg', 1)} | {span(cmd, 'max_tilt_deg', 1)} | {span(cmd, 'min_z', 3)} "
              f"| {span(cmd, 'single_support_frac', 2)} | {span(cmd, 'nonfoot_ground', 0)} "
              f"| {span(stop, 'stop_time_s', 2)} | {span(stop, 'stop_dist_max_m', 3)} | {span(stop, 'stop_yaw_deg', 1)} "
              f"| {','.join(final)} |")


if __name__ == '__main__':
    args = sys.argv[1:]
    seg = '1'
    if '--segment' in args:
        i = args.index('--segment')
        seg = args[i + 1]
        del args[i:i + 2]
    main(args[0], seg, str(int(seg) + 1))

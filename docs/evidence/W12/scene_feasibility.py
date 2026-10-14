"""W11 pre-check (ADR-015): free-space connectivity of static_world layouts under frozen
W10 radii. Pure geometry from the scene YAML; read-only, no simulation, no map writing."""
import sys
from pathlib import Path
import numpy as np
import yaml
from scipy import ndimage

RES = 0.025


def clearance_fn(layout):
    xmin, xmax, ymin, ymax = layout['bounds']
    boxes = [(b['center'][0], b['center'][1], b['size'][0], b['size'][1]) for b in layout['boxes']]

    def clear(x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        terms = [x-xmin, xmax-x, y-ymin, ymax-y]
        for cx, cy, sx, sy in boxes:
            terms.append(np.hypot(np.maximum(abs(x-cx)-sx/2, 0), np.maximum(abs(y-cy)-sy/2, 0)))
        return np.minimum.reduce(terms)
    return clear


def report(layout, points=(), legs=()):
    clear = clearance_fn(layout)
    xmin, xmax, ymin, ymax = layout['bounds']
    X, Y = np.meshgrid(np.arange(xmin, xmax, RES)+RES/2, np.arange(ymin, ymax, RES)+RES/2, indexing='ij')
    d = clear(X, Y)
    cell = lambda p: (int((p[0]-xmin)/RES), int((p[1]-ymin)/RES))
    a, b = layout['initial_pose'][:2], layout['goal'][:2]
    out = {}
    for r in (0.7, 1.2, 1.65, 1.70, 1.95, 2.0):
        lab, n = ndimage.label(d > r)
        sizes = sorted(((lab == i).sum()*RES*RES for i in range(1, n+1)), reverse=True)[:3]
        out[r] = dict(connected=bool(lab[cell(a)] > 0 and lab[cell(a)] == lab[cell(b)]),
                      components=n, largest_m2=[round(float(s), 2) for s in sizes])
    t = np.linspace(0, 1, 400)
    straight = float(clear(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t).min())
    return dict(radii=out, straight_AB_min_clearance=round(straight, 3),
                points={k: round(float(clear(*p)), 3) for k, p in points},
                legs=[[list(p), list(q), round(float(clear(p[0]+(q[0]-p[0])*t, p[1]+(q[1]-p[1])*t).min()), 3)]
                      for p, q in legs])


if __name__ == '__main__':
    import json
    layout = yaml.safe_load(Path(sys.argv[1]).read_text())
    print(json.dumps(report(layout), indent=2))


def check_cases(cases_path, scene_path, approach=1.0, minimum=2.25):
    """Clearance of every registered start, goal and approach waypoint (W11 C2)."""
    layout = yaml.safe_load(Path(scene_path).read_text())
    clear = clearance_fn(layout)
    data = yaml.safe_load(Path(cases_path).read_text())
    rows = []
    entries = [(c['id'], c['start'], c['goal']) for c in data['cases']]
    cr = data['cancel_replace']
    entries += [('cancel_replace', cr['start'], g) for g in (cr['first_goal'], cr['second_goal'], cr['replacement_goal'])]
    for cid, start, goal in entries:
        import math
        pre = (goal[0] - approach*math.cos(goal[2]), goal[1] - approach*math.sin(goal[2]))
        t = np.linspace(0, 1, 50)
        segment = float(clear(pre[0] + (goal[0]-pre[0])*t, pre[1] + (goal[1]-pre[1])*t).min())
        straight = float(clear(start[0] + (goal[0]-start[0])*t, start[1] + (goal[1]-start[1])*t).min())
        values = dict(start=float(clear(*start[:2])), goal=float(clear(*goal[:2])), approach_segment=segment)
        rows.append(dict(id=cid, start=start, goal=goal, approach_waypoint=list(pre),
                         clearance_m={k: round(v, 3) for k, v in values.items()},
                         straight_line_min_clearance_m=round(straight, 3), straight_line_blocked=straight < 1.65,
                         ok=all(v >= minimum for v in values.values())))
    return dict(minimum_m=minimum, approach_m=approach, cases=rows, verdict='PASS' if all(r['ok'] for r in rows) else 'FAIL')

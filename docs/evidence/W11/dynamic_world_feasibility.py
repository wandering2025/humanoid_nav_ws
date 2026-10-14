"""ADR-017 (pre-W12): dynamic_crossing_world v2 feasibility under frozen W10/W11 radii.
Pure geometry from the scene YAML with each W12 obstacle slot placed on the A-B line or at
its region centre. Read-only; no simulation, no map. Run from $WS with the venv python."""
import json, math, sys
sys.path.insert(0, 'docs/evidence/W11')
import numpy as np, yaml
from scipy import ndimage
import scene_feasibility as sf
path = 'src/t1_mujoco/worlds/dynamic_crossing_world.yaml'
base = yaml.safe_load(open(path))
regions = {r['name']: r['bounds'] for r in base['activity_regions']}
slots = {s['name']: s for s in base['obstacle_slots']}
A, B = base['initial_pose'], base['goal']
t = np.linspace(0, 1, 400)
def with_box(name, cx, cy):
    d = json.loads(json.dumps(base)); s = slots[name]
    d['boxes'].append(dict(name=name, center=[cx, cy, .75], size=s['size']))
    return d
def summary(layout, label):
    clear = sf.clearance_fn(layout)
    xmin, xmax, ymin, ymax = layout['bounds']; res = .025
    X, Y = np.meshgrid(np.arange(xmin, xmax, res)+res/2, np.arange(ymin, ymax, res)+res/2, indexing='ij')
    d = clear(X, Y); cell = lambda p: (int((p[0]-xmin)/res), int((p[1]-ymin)/res))
    conn = {}
    for r in (1.65, 1.95, 2.05, 2.25):
        lab, _ = ndimage.label(d > r); conn[r] = bool(lab[cell(A)] > 0 and lab[cell(A)] == lab[cell(B)])
    straight = float(clear(A[0]+(B[0]-A[0])*t, A[1]+(B[1]-A[1])*t).min())
    pre = (B[0]-1.5, B[1])
    return dict(case=label, A=round(float(clear(*A[:2])), 3), B=round(float(clear(*B[:2])), 3),
                approach_segment_B=round(float(clear(pre[0]+(B[0]-pre[0])*t, 0*t).min()), 3),
                straight_AB_min=round(straight, 3), connected=conn,
                north_passage_min=round(float(clear(np.linspace(1.5, 7.5, 200), 2.75+0*np.linspace(0, 1, 200)).min()), 3),
                south_passage_min=round(float(clear(np.linspace(1.5, 7.5, 200), -2.75+0*np.linspace(0, 1, 200)).min()), 3))
out = [summary(base, 'no_dynamic_obstacle')]
bx = sum(regions['blocking'][:2])/2; by = sum(regions['blocking'][2:])/2
out.append(summary(with_box('blocking_slot', bx, by), 'blocker_parked_at_region_centre'))
cx = sum(regions['crossing'][:2])/2
out.append(summary(with_box('crossing_slot', cx, 0.0), 'crosser_on_A_B_line'))
ax = sum(regions['approach'][:2])/2
out.append(summary(with_box('approach_slot', ax, 0.0), 'approacher_on_A_B_line'))
# Distances that W12 timing must respect (robot on the A-B line, centre).
info = dict(regions=regions, frozen=dict(R_stop=1.65, R_slow=1.95, nav_radius=2.05, nav_vx_max=0.45,
            brake_at_0_45_m=0.245, note='slow zone acts as stop: <=0.45 m/s halved falls in W05 deadband'),
            approach_region_lateral_edge_to_AB_line_m=regions['approach'][3])
result = dict(scene_yaml=path, cases=out, info=info)
open('docs/evidence/W11/adr017-dynamic-world-feasibility.json', 'w').write(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))

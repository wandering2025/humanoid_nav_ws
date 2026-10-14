"""Pre-map dimensions, ray/contact participation, route clearance; explicit venv."""
import json
import os
from pathlib import Path

import mujoco
import numpy as np

from t1_mujoco.scenes import assemble, load_layout
from scene_feasibility import report
from remap import ROUTE

WS = Path(os.environ['T1_WS'])
scene = WS/'src/t1_mujoco/worlds/dynamic_crossing_world.yaml'
layout = load_layout(scene)
rows = []
for slot in [None] + [s['name'] for s in layout['obstacle_slots']]:
    path, provenance = assemble(scene, '/home/haoyang/robot_sim_sources/booster_assets',
                                WS/'run_data/w12_scene_cache', slot)
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    item = dict(slot=slot, nq=model.nq, nv=model.nv, nu=model.nu, nmocap=model.nmocap)
    if slot:
        data.mocap_pos[0] = [6, 3, .75]
        mujoco.mj_forward(model, data)
        hit = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(model, data, np.array([3., 3., .75]), np.array([1., 0., 0.]),
                                 np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8), 1, -1, hit)
        geom = model.geom('obstacle_'+slot)
        item.update(ray_distance_m=float(distance), ray_hit=mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(hit[0])),
                    collision_enabled=int(geom.contype[0]) == 1 and int(geom.conaffinity[0]) == 1,
                    ray_hits_obstacle=int(hit[0]) == geom.id)
    item['ok'] = ((model.nq, model.nv, model.nu) == (30, 29, 23)
                  and model.nmocap == (1 if slot else 0)
                  and (not slot or item['collision_enabled'] and item['ray_hits_obstacle']))
    rows.append(item)
points = [('A', (0, 0))] + [(f'route{i}', p) for i, p in enumerate(ROUTE)]
legs = list(zip([(0, 0)] + ROUTE[:-1], ROUTE))
route = report(layout, points, legs)
route_ok = min(route['points'].values()) >= 2.25 and min(r[2] for r in route['legs']) >= 2.25
result = dict(models=rows, route=route, route_clearance_225cm=route_ok,
              verdict='PASS' if all(r['ok'] for r in rows) and route_ok else 'FAIL')
(WS/'docs/evidence/W12/c1-scene.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
raise SystemExit(0 if result['verdict'] == 'PASS' else 1)

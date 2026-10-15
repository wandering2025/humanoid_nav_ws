import json,math,sys
from pathlib import Path
w=Path('/home/haoyang/humanoid_nav_ws_advanced')
for ident in sys.argv[1:]:
 p=w/'run_data/advanced'/ident
 r=json.loads((p/'final-result.json').read_text());o=json.loads((p/'observation.json').read_text());rows=o['rows'];pts=[]
 for row in rows:
  if not pts or row['sim']-pts[-1]['sim']>=.5:pts.append(row)
 distance=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(pts,pts[1:]))
 out=dict(run=ident,code=r['code_sha'],status=r['status'],duration_sim=rows[-1]['sim']-rows[0]['sim'],sampled_distance_m=distance,
 goals=[dict(label=g['label'],duration=g['result'].get('duration_sim'),status=g['result'].get('status')) for g in r['goals']],
 dynamic=r.get('dynamic'),contacts=r.get('contacts'),fell=r.get('fell'),error=r.get('error'),nonstraight=r.get('nonstraight_plan'),
 map_known=[r['map_growth'][0]['known_m2'],r['map_growth'][-1]['known_m2']],cleanup=r.get('dds_port_released'),slam_shutdown=r.get('slam_shutdown'))
 print(json.dumps(out,indent=2))

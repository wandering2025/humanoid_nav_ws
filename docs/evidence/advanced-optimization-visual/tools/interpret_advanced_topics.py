import json,hashlib,shutil
from pathlib import Path
w=Path('/home/haoyang/humanoid_nav_ws_advanced');out=w/'docs/evidence/advanced-optimization-visual';raw=out/'raw/analysis'
p=Path('/tmp/advanced-opt-visual-topics03.json');r=json.loads(p.read_text());rows=r['records']
planner=[x for x in rows if x['topic']=='planner'];display=[x for x in rows if x['topic']=='display'];overview=[x for x in rows if x['topic']=='overview']
pairs=[]
for x in display:
 candidates=[y for y in planner if y['geometry_sha256']==x['geometry_sha256']]
 assert candidates
 y=min(candidates,key=lambda y:abs(x['wall']-y['wall']))
 pairs.append(dict(display_sha256=x['sha256'],planner_sha256=y['sha256'],geometry_sha256=x['geometry_sha256'],lag_s=x['wall']-y['wall'],serialized_equal=x['sha256']==y['sha256']))
assert len(display)==23 and len(overview)==1 and r['overview_geometry_never_displayed_as_execution']
assert all(-.1<=x['lag_s']<.25 for x in pairs)
c=json.loads(Path('/tmp/advanced-cdr-roundtrip.json').read_text())
assert all(x['decoded_messages_all_equal'] for x in c) and any(x['distinct_serializations']>1 for x in c)
for q in (p,Path('/tmp/advanced-cdr-roundtrip.json')):shutil.copy2(q,raw/q.name)
s=dict(run_id=r['run_id'],source_sha=r['source_sha'],status='PASS_GEOMETRY_AND_OVERVIEW_PROVENANCE',initial_strict_byte_audit='FAILED; retained unmodified. Exact-byte identity is not a valid necessary condition: CPU experiment reproduces unequal CDR bytes40..43 for equal decoded ROS messages.',display_paths=len(display),planner_paths=len(planner),overview_paths=len(overview),all_display_geometry_and_frame_match_planner=True,matched_serialized_paths=sum(x['serialized_equal'] for x in pairs),max_observed_delivery_lag_s=max(x['lag_s'] for x in pairs),overview_never_displayed_as_execution=True,pairs=pairs,cdr_roundtrip=c,scope='All23 display trajectories match map frame and every pose position/orientation;11 additionally match whole serialized bytes. Remaining per-pose/header timestamp fields were not separately captured. Actual callback republishes the received message unchanged; CPU callback test checks equality. Serialization-only criterion is superseded based on reproducible no-GPU evidence, not silently marked PASS.')
(out/'live-path-correspondence.json').write_text(json.dumps(s,indent=2)+'\n')
print({k:v for k,v in s.items() if k not in ('pairs','cdr_roundtrip')})

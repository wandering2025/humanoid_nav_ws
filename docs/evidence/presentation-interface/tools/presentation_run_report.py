import sys,json,hashlib,math
from pathlib import Path
w=Path('/home/haoyang/humanoid_nav_ws_advanced');ident,label=sys.argv[1:3];run=w/'run_data/advanced'/ident;out=w/'docs/evidence/presentation-interface';out.mkdir(exist_ok=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=json.loads((run/'final-result.json').read_text());manifest=json.loads((w/'run_data'/ident/'manifest.json').read_text());goal=json.loads((run/'presentation-goal.json').read_text())
assert r['status']=='SUCCESS' and r['presentation_goal_consistent'] and r['presentation_goal']==manifest['presentation_goal']==goal
assert manifest['goal']==goal['goal']==r['nav2']['goal']==r['nav2']['verified']['target']
approach=next(x for x in r['goals'] if x['label']=='NAVIGATING_APPROACH')
assert approach['pose']==goal['navigation_approach']
end=r['a_to_b_overview']['points'][-1];assert math.dist(end,goal['goal'][:2])<.075
assert r['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0) and not r['fell']
assert r['dynamic']['verdict']=='PASS' and r['safe_stop']['physical_stationary'] and r['dds_port_released']
assert not r['remaining_owned_pids'] and not r['remaining_project_processes']
assert all(v==0 for v in r['parent_exit_codes'].values()) and all(v['exit_code']==0 for v in r['launch_process_exit_codes'].values())
keys=('code_sha','dirty','status','presentation_goal_consistent','contacts','fell','body_clearance','settle','controller_plugin','controller_goal_tolerances','mapping_passes','navigation_static_map_unchanged','dynamic','nav2','safe_stop','slam_shutdown','graph_contract','parent_exit_codes','launch_process_exit_codes','remaining_owned_pids','remaining_project_processes','dds_port_released','error','gpu_monitor_error')
summary={k:r.get(k) for k in keys};summary.update(run_id=ident,presentation_goal=goal,overview_length_m=r['a_to_b_overview']['length'],overview_endpoint=end,derived_approach_received=approach['pose'],artifacts={str(p.relative_to(w)):sha(p) for p in [run/'final-result.json',run/'presentation-goal.json',w/'run_data'/ident/'manifest.json']})
samples=run/'gpu/gpu-samples.jsonl'
if samples.exists():
 rows=[json.loads(s) for s in samples.read_text().splitlines()];summary['gpu_peak']={k:max(v[k] for v in rows) for k in ('temperature_c','memory_mib','utilization_pct')}
summary['screenshots']={p.name:dict(path=str(p),sha256=sha(p)) for p in run.glob('*.png')}
(out/(label+'.json')).write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({k:summary.get(k) for k in ('run_id','status','presentation_goal_consistent','overview_endpoint','derived_approach_received','dynamic','nav2')},indent=2))

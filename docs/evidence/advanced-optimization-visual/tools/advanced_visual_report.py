import argparse,hashlib,json,math,subprocess,ast
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('run_id');p.add_argument('label');a=p.parse_args()
w=Path('/home/haoyang/humanoid_nav_ws_advanced');run=w/'run_data/advanced'/a.run_id
out=w/'docs/evidence/advanced-optimization-visual'
sha=lambda q:hashlib.sha256(q.read_bytes()).hexdigest()
r=json.loads((run/'final-result.json').read_text())
keys=('code_sha','status','startup_check_completed','zero_policy','arrived','contacts','fell','body_clearance','settle','controller_goal_tolerances','controller_plugin','mapping_passes','mapping_completed_sim','navigation_start_sim','navigation_static_map_unchanged','mapping_coverage_checks','slam_measurement_transitions','nonstraight_plan','dynamic','safe_stop','slam_shutdown','graph_contract','parent_exit_codes','launch_process_exit_codes','dds_port_released','remaining_owned_pids','remaining_project_processes','shutdown_signals','error','cleanup_error','gpu_monitor_error','gpu_summary','nav2')
s={k:r.get(k) for k in keys};s['run_id']=a.run_id
s['final_result_sha256']=sha(run/'final-result.json')
samples=[json.loads(l) for l in (run/'gpu/gpu-samples.jsonl').read_text().splitlines()]
s['gpu_observed']={k:dict(min=min(v[k] for v in samples),max=max(v[k] for v in samples)) for k in ('temperature_c','memory_mib','utilization_pct')}
s['gpu_baseline_memory_mib']=samples[0]['memory_mib']
s['gpu_peak_growth_mib']=s['gpu_observed']['memory_mib']['max']-samples[0]['memory_mib']
checks=json.loads((run/'gpu/host/windows-checks.json').read_text())
s['windows_checks']=dict(count=len(checks),events=[e for row in checks for e in row['Events']])
s['graphics_halt_created']=(run/'gpu/halt.json').exists()
s['screenshots']={q.name:dict(bytes=q.stat().st_size,sha256=sha(q)) for q in sorted(run.glob('*.png'))}
base='2875228d3c27e293c80087453543be0c549f3347'
changed=subprocess.check_output(['git','diff','--name-only',base,r['code_sha'],'--','scripts','src'],cwd=w,text=True).splitlines()
s['runtime_source_matches_headless_implementation']=not changed
s['changes_since_headless_implementation']=changed
assert set(changed)<=set(['scripts/advanced/demo.py','scripts/advanced/test_contract.py','scripts/demo/gpu_watch.py','scripts/demo/test_gpu_watch.py']),changed
def without_visual_markers(commit):
 text=subprocess.check_output(['git','show',commit+':scripts/advanced/demo.py'],cwd=w,text=True)
 text=text.replace(',idle_max_utilization_pct=60.','').replace('            self.overview_geometry=path_geometry(path)\n','')
 tree=ast.parse(text)
 tree.body=[n for n in tree.body if not isinstance(n,ast.FunctionDef) or n.name!='path_geometry']
 for node in tree.body:
  if isinstance(node,ast.ClassDef) and node.name=='Advanced':node.body=[m for m in node.body if not isinstance(m,ast.FunctionDef) or m.name not in ('markers','live_plan')]
 return ast.dump(tree,include_attributes=False)
s['navigation_and_safety_identical_to_headless_implementation']=without_visual_markers(base)==without_visual_markers(r['code_sha'])
assert s['navigation_and_safety_identical_to_headless_implementation']
o=json.loads((run/'observation.json').read_text());rows=o['rows'];sample=[]
for row in rows:
 if not sample or row['sim']-sample[-1]['sim']>=.5:sample.append(row)
s['trace_duration_sim_s']=rows[-1]['sim']-rows[0]['sim']
s['sampled_trajectory_m']=sum(math.hypot(b['x']-x['x'],b['y']-x['y']) for x,b in zip(sample,sample[1:]))
s['goals']=r.get('goals',[])
s['overview_length_m']=r.get('a_to_b_overview',{}).get('length')
growth=r.get('map_growth',[])
s['known_area_m2']=[growth[0]['known_m2'],growth[-1]['known_m2']] if growth else []
phases=[]
for v in o['obstacles']:
 if not phases or phases[-1]['phase']!=v['phase']:phases.append(dict(phase=v['phase'],sim=v['sim_time'],position=v['position']))
s['obstacle_phases']=phases
f=r.get('nav2',{})
if f.get('accepted_sim') is not None:
 seg=[v for v in sample if f['accepted_sim']<=v['sim']<=f['end_sim']]
 if len(seg)>1:
  dist=sum(math.hypot(b['x']-x['x'],b['y']-x['y']) for x,b in zip(seg,seg[1:]));direct=math.hypot(seg[-1]['x']-seg[0]['x'],seg[-1]['y']-seg[0]['y']);angles=[seg[0]['yaw']]
  for x,b in zip(seg,seg[1:]):angles.append(angles[-1]+math.atan2(math.sin(b['yaw']-x['yaw']),math.cos(b['yaw']-x['yaw'])))
  s['terminal_trajectory']=dict(travel_m=dist,endpoint_distance_m=direct,distance_ratio=dist/direct,heading_span_rad=max(angles)-min(angles))
(out/(a.label+'.json')).write_text(json.dumps(s,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps({k:s.get(k) for k in ('run_id','code_sha','status','dynamic','gpu_observed','gpu_peak_growth_mib','terminal_trajectory','sampled_trajectory_m','trace_duration_sim_s','dds_port_released')},indent=2))

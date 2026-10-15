from pathlib import Path
import argparse,json,hashlib,subprocess,math,shutil,ast
p=argparse.ArgumentParser();p.add_argument('--intervention',required=True);p.add_argument('--control',required=True);p.add_argument('--interrupt',required=True);args=p.parse_args()
w=Path('/home/haoyang/humanoid_nav_ws_advanced');out=w/'docs/evidence/advanced-optimization';out.mkdir(exist_ok=True,parents=True)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def git(*args,cwd=w):return subprocess.check_output(['git',*args],cwd=cwd,text=True).strip()
def write(name,value):(out/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
def summary(ident):
 run=w/'run_data/advanced'/ident;r=json.loads((run/'final-result.json').read_text());o=json.loads((run/'observation.json').read_text());rows=o['rows'];sample=[]
 for row in rows:
  if not sample or row['sim']-sample[-1]['sim']>=.5:sample.append(row)
 result={k:r.get(k) for k in ('run_id','code_sha','status','control_no_obstacle','arrived','contacts','fell','body_clearance','settle','controller_goal_tolerances','controller_plugin','mapping_passes','map_route_checks','navigation_start_sim','navigation_map_sha256','navigation_static_map_unchanged','mapping_coverage_checks','slam_measurement_transitions','nonstraight_plan','dynamic','safe_stop','slam_shutdown','graph_contract','parent_exit_codes','launch_process_exit_codes','dds_port_released','remaining_owned_pids','remaining_project_processes','shutdown_signals','error','cleanup_error','gpu_monitor_error')}
 result.update(run_id=ident,physical_trace_duration_sim_s=rows[-1]['sim']-rows[0]['sim'] if rows else None,
  sampled_trajectory_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(sample,sample[1:])),
  goals=r['goals'],map_growth=dict(initial_known_m2=r['map_growth'][0]['known_m2'],final_known_m2=r['map_growth'][-1]['known_m2']) if r['map_growth'] else {},
  overview_length_m=r.get('a_to_b_overview',{}).get('length'),final_result_sha256=sha(run/'final-result.json'),observation_sha256=sha(run/'observation.json'))
 final=r.get('nav2',{})
 if final.get('accepted_sim') is not None:
  segment=[x for x in sample if final['accepted_sim']<=x['sim']<=final['end_sim']]
  if len(segment)>1:
   distance=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(segment,segment[1:]));angles=[segment[0]['yaw']]
   for a,b in zip(segment,segment[1:]):angles.append(angles[-1]+math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])))
   direct=math.hypot(segment[-1]['x']-segment[0]['x'],segment[-1]['y']-segment[0]['y'])
   result['terminal_trajectory']=dict(sample_interval_min_s=.5,travel_m=distance,endpoint_distance_m=direct,distance_ratio=distance/direct if direct else None,heading_span_rad=max(angles)-min(angles),scope='Physical odom trajectory from final goal-tool acceptance through verified standstill; detects loops independently of Nav2 result/retry count.')
 phases=[]
 for v in o['obstacles']:
  if not phases or phases[-1]['phase']!=v['phase']:phases.append(dict(phase=v['phase'],sim=v['sim_time'],position=v['position']))
 result['obstacle_phases']=phases
 return result
intervention=summary(args.intervention);control=summary(args.control);interrupt=summary(args.interrupt)
assert intervention['status']=='SUCCESS' and intervention['dynamic']['verdict']=='PASS'
assert control['status']=='SUCCESS' and control['control_no_obstacle']
for full in (intervention,control):
 assert full['terminal_trajectory']['distance_ratio']<1.5 and full['terminal_trajectory']['heading_span_rad']<math.pi/2,full['terminal_trajectory']
assert intervention['code_sha']==control['code_sha']
assert interrupt['code_sha']==intervention['code_sha']
assert interrupt['status']=='FAILED' and interrupt['safe_stop']['physical_stationary']
assert all(v==0 for v in interrupt['parent_exit_codes'].values()) and all(v['exit_code']==0 for v in interrupt['launch_process_exit_codes'].values())
assert interrupt['dds_port_released'] and not interrupt['remaining_owned_pids'] and not interrupt['remaining_project_processes']
assert interrupt['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0) and not interrupt['fell']
interrupt['interrupt_request']=json.loads((w/'run_data/advanced'/args.interrupt/'interrupt-request.json').read_text())
assert float(interrupt['interrupt_request']['request']['observed_motion']['u_pol_x'])>.1 and interrupt['interrupt_request']['wrapper_exit']!=0
write('intervention.json',intervention);write('control.json',control);write('interrupt.json',interrupt)
coverage=json.loads((w/'run_data/advanced'/args.intervention/'map-coverage.json').read_text());write('map-coverage.json',coverage)
topics=json.loads(Path('/tmp/advanced-live-topics-final.json').read_text());assert topics['matched_planner_display_hashes']>=1
assert topics['run_id']==args.intervention and topics['source_sha']==intervention['code_sha']
write('live-path-topics.json',topics)
prior=summary('20261015T180213Z_advanced_c20b40f2-12f5-4b5c-8c96-0d5a68461b8f')
write('comparison.json',dict(old_run=prior['run_id'],new_run=args.intervention,
 old_distance_m=prior['sampled_trajectory_m'],new_distance_m=intervention['sampled_trajectory_m'],
 distance_reduction_fraction=1-intervention['sampled_trajectory_m']/prior['sampled_trajectory_m'],
 old_trace_sim_s=prior['physical_trace_duration_sim_s'],new_trace_sim_s=intervention['physical_trace_duration_sim_s'],
 intervention_approach_s=next(x['result']['duration_sim'] for x in intervention['goals'] if x['label']=='NAVIGATING_APPROACH'),
 control_approach_s=next(x['result']['duration_sim'] for x in control['goals'] if x['label']=='NAVIGATING_APPROACH'),
 caveat='Actual 0.5s trajectory samples; workflows, B and cart differ from old release. Not a matched planner benchmark. Gait/timing variation also affects total duration; same-source control is evidence of obstacle-response contrast, not a reliability study.'))
source=intervention['code_sha'];changed=git('diff','--name-only',source,'--','scripts','src').splitlines()
assert not changed,changed
# The moving interruption uses the identical final source; also audit the
# cancellation/cleanup methods explicitly for evidence provenance.
def methods(text):
 tree=ast.parse(text)
 advanced=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Advanced')
 return {m.name:ast.dump(m,include_attributes=False) for m in advanced.body if isinstance(m,ast.FunctionDef)}
interrupted_source=subprocess.check_output(['git','show',interrupt['code_sha']+':scripts/advanced/demo.py'],cwd=w,text=True)
for name in ('mapping_pass','cleanup','spin'):
 assert methods(interrupted_source)[name]==methods((w/'scripts/advanced/demo.py').read_text())[name]
protected=['scripts/run_demo.sh','src/t1_navigation/config/safety.yaml','src/t1_navigation/config/navigation.yaml','src/t1_mujoco/config/command_map.yaml','src/t1_bringup/t1_bringup/navigation_adapter.py','src/t1_bringup/t1_bringup/safety.py','src/t1_mujoco/t1_mujoco/runner.py']
goal_tool=w/'src/t1_bringup/t1_bringup/navigation_tools.py'
compatible=goal_tool.read_text().replace("    parser.add_argument('--require-clock', action='store_true', help='advanced opt-in: wait for the first positive simulation timestamp')\n",'').replace("        if options.require_clock and not spin_until(lambda: sim() > 0., 10.):\n            raise RuntimeError('positive simulation clock unavailable before goal setup')\n",'')
assert compatible.encode()==subprocess.check_output(['git','show','demo-v0.1.0:src/t1_bringup/t1_bringup/navigation_tools.py'],cwd=w)
assert intervention['goals'][-1]['result']['accepted_sim']>intervention['navigation_start_sim']
assert control['goals'][-1]['result']['accepted_sim']>control['navigation_start_sim']
for name in protected:assert (w/name).read_bytes()==subprocess.check_output(['git','show','demo-v0.1.0:'+name],cwd=w)
original=Path('/home/haoyang/humanoid_nav_ws');assert git('rev-parse','HEAD',cwd=original)=='0f51d54b1dbfdde0664966118b5546109a3f6ff2' and not git('status','--porcelain',cwd=original)
assert git('rev-parse','demo-v0.1.0')=='844036e69644272dce6178552dd10778e80d4d35'
assert git('rev-parse','demo-v0.2.0')=='19466fc53775d30d96b5b76c0d9a5b2a440d0c95'
assert git('rev-parse','demo-v0.2.0^{}')=='b191ef6af36b0a3d611604cd9de922866167ce27'
external={}
for name,path,expected in [('deploy','/home/haoyang/robot_sim_sources/booster_deploy','7bb1462e2ac742eba5c01659b815e2e99fbe617f'),('assets','/home/haoyang/robot_sim_sources/booster_assets','12a97516f57469078707afb587a5f84c1cb110d1')]:
 assert git('rev-parse','HEAD',cwd=path)==expected and not git('status','--porcelain',cwd=path)
 external[name]=dict(head=expected,clean=True)
assert (w/'run_data/demo-gpu-halt.json').read_bytes()==(w/'run_data/advanced/20261015T220015Z_advanced_c171be81-40f2-4a13-a537-2aa174bce3f5/gpu/halt.json').read_bytes()
graphics=['scripts/demo/gpu_watch.py','scripts/demo/cpu_guard.py','scripts/demo/build_rviz.sh']
for name in graphics:assert (w/name).read_bytes()==subprocess.check_output(['git','show','b191ef6af36b0a3d611604cd9de922866167ce27:'+name],cwd=w)
write('source-audit.json',dict(graphics_source_unchanged={n:sha(w/n) for n in graphics},external=external,tested_source_sha=source,delivery_head_at_audit=git('rev-parse','HEAD'),runtime_source_unchanged=True,interruption_source_sha=interrupt['code_sha'],interruption_methods_identical=True,interruption_scope='First moving mapping goal at the identical final source, including the RPP controller; expected mission failure, physical safety stop and cleanup verified.',
 stable_original_head=git('rev-parse','HEAD',cwd=original),stable_original_clean=True,
 tags={tag:dict(object=git('rev-parse',tag),commit=git('rev-parse',tag+'^{}')) for tag in ('demo-v0.1.0','demo-v0.2.0')},
 stable_byte_identical={n:sha(w/n) for n in protected},
 goal_tool_default_compatible=True,advanced_clock_guard='Opt-in --require-clock only; stripping 3 exact lines reproduces stable source. Actual final-goal acceptance timestamps verified positive and after navigation start.',
 advanced_controller=dict(plugin=intervention['controller_plugin'],config_sha256=sha(w/'src/t1_navigation/config/advanced_controller.yaml'),goal_tolerances=intervention['controller_goal_tolerances'],scope='Advanced-only RPP override; baseline MPPI and original0.20/0.20 tolerances unchanged'),
 gpu_latch=dict(path=str(w/'run_data/demo-gpu-halt.json'),sha256=sha(w/'run_data/demo-gpu-halt.json'),preserved=True),
 graphical_validation='PENDING_GPU_RECOVERY_AUTHORIZATION',new_video=None))
# All attempts starting with this optimization, plus the pre-existing user GPU halt.
records=[];index=[]
for run in sorted((w/'run_data/advanced').glob('*_advanced_*')):
 if run.name<'20261015T220015Z':continue
 f=run/'final-result.json'
 if f.exists():
  r=json.loads(f.read_text());records.append(dict(run_id=run.name,code_sha=r.get('code_sha'),status=r.get('status'),error=r.get('error'),contacts=r.get('contacts'),fell=r.get('fell'),dds_port_released=r.get('dds_port_released'),result_sha256=sha(f)))
 else:records.append(dict(run_id=run.name,status='NO_FINAL_RESULT',scope='Preflight or incomplete run directory; preserved, not counted as success'))
 for root in (run,w/'run_data'/run.name):
  if root.exists():
   for q in sorted(root.rglob('*')):
    if q.is_file():index.append(dict(path=str(q.relative_to(w)),bytes=q.stat().st_size,sha256=sha(q)))
write('attempts.json',records);write('raw-index.json',dict(files=index,scope='Existing failed attempts are preserved byte-for-byte; runtime artifacts stay ignored. Includes pre-existing user GPU halt and optimization attempts.'))
raw=out/'raw/analysis';raw.mkdir(exist_ok=True,parents=True)
for name in ('package_advanced_opt.py','advanced_opt_plot.py','advanced_opt_report.py','advanced_map_report.py','advanced_map_plot.py','advanced_opt_interrupt.py','advanced_live_topics.py','watch_advanced_topics.py'):
 q=Path('/tmp')/name
 if q.exists():shutil.copy2(q,raw/name)
for name in ('advanced-opt-final-build.log','advanced-opt-final-tests.log','advanced-opt-cpu-provenance.json','advanced-live-topics-wrong-discovery.json','advanced-live-topics.json','advanced-live-topics-b229.json'):
 q=Path('/tmp')/name
 if q.exists():shutil.copy2(q,raw/name)
cpu=json.loads(Path('/tmp/advanced-opt-cpu-provenance.json').read_text());assert cpu['source_sha']==source
write('cpu-checks.json',cpu)
write('analysis-tools.json',{str(q.relative_to(w)):sha(q) for q in sorted(raw.iterdir()) if q.is_file()})
print(json.dumps(dict(source=source,intervention=args.intervention,control=args.control,interrupt=args.interrupt,attempts=len(records),files=len(index)),indent=2))

import sys,json,hashlib,subprocess,shutil,os
from pathlib import Path
from datetime import datetime,timezone
w=Path('/home/haoyang/humanoid_nav_ws_advanced');out=w/'docs/evidence/presentation-interface';out.mkdir(exist_ok=True)
default,changed,*minimal=sys.argv[1:]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def git(*args,cwd=w):return subprocess.check_output(['git',*args],cwd=cwd,text=True).strip()
def write(name,obj):(out/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
for ident in (default,changed,*minimal):
 r=json.loads((w/'run_data/advanced'/ident/'final-result.json').read_text());assert r['status']=='SUCCESS' and r['presentation_goal_consistent']
 assert not git('diff','--name-only',r['code_sha'],'--','scripts','src')
marker=json.loads(Path('/tmp/presentation-changed-marker-audit.json').read_text());assert marker['status']=='PASS' and marker['goal'][:2]==[11.,.5]
write('changed-marker-audit.json',marker)
original=Path('/home/haoyang/humanoid_nav_ws');assert git('rev-parse','HEAD',cwd=original)=='0f51d54b1dbfdde0664966118b5546109a3f6ff2' and not git('status','--porcelain',cwd=original)
tags={name:dict(object=git('rev-parse',name),commit=git('rev-parse',name+'^{}')) for name in ('demo-v0.1.0','demo-v0.2.0','demo-v0.2.1')}
assert tags['demo-v0.1.0']['object']=='844036e69644272dce6178552dd10778e80d4d35'
assert tags['demo-v0.2.0']['object']=='19466fc53775d30d96b5b76c0d9a5b2a440d0c95'
assert tags['demo-v0.2.1']['object']=='c0ad96785f77370f461ccbb6e063bf26e870b998'
protected=['scripts/run_demo.sh','src/t1_navigation/config/safety.yaml','src/t1_navigation/config/navigation.yaml','src/t1_mujoco/config/command_map.yaml','src/t1_bringup/t1_bringup/navigation_adapter.py','src/t1_bringup/t1_bringup/safety.py','src/t1_mujoco/t1_mujoco/runner.py']
for name in protected:assert (w/name).read_bytes()==subprocess.check_output(['git','show','demo-v0.1.0:'+name],cwd=w)
for name in ('src/t1_mujoco/worlds/advanced_world.yaml','src/t1_bringup/config/advanced_events.yaml','src/t1_navigation/config/advanced_controller.yaml','scripts/demo/gpu_watch.py'):
 assert (w/name).read_bytes()==subprocess.check_output(['git','show','demo-v0.2.1:'+name],cwd=w)
# Auditable simulator delta: only advanced metadata loading and marker arguments.
actual=(w/'src/t1_mujoco/t1_mujoco/simulation.py').read_text()
actual=actual.replace("        presentation_goal = None\n        if p['mode'] == 'advanced_mapping':\n            from .advanced_goal import runtime_goal\n            presentation_goal = runtime_goal()\n",'')
actual=actual.replace("        if presentation_goal is not None:\n            log.manifest.update(goal=presentation_goal['goal'], presentation_goal=presentation_goal)\n            log.save()\n",'')
actual=actual.replace("                    if presentation_goal is not None:\n                        annotate(viewer, goal=presentation_goal['goal'])\n                    else:\n                        annotate(viewer)","                    annotate(viewer)")
assert actual.encode()==subprocess.check_output(['git','show','demo-v0.2.1:src/t1_mujoco/t1_mujoco/simulation.py'],cwd=w)
external={}
for name,expected in [('booster_deploy','7bb1462e2ac742eba5c01659b815e2e99fbe617f'),('booster_assets','12a97516f57469078707afb587a5f84c1cb110d1')]:
 path='/home/haoyang/robot_sim_sources/'+name
 assert git('rev-parse','HEAD',cwd=path)==expected and not git('status','--porcelain',cwd=path)
 external[name]=expected
file=w/'presentation/goal.yaml';assert sha(file)=='908b1e7c90f499461e9c49b4dbb513e8920e41531987511e3d11c46703cae0b0'
os.environ.update(T1_DEMO_DIR='/tmp/presentation-final-audit',T1_DEMO_RUN_ID='presentation-final-audit');sys.path.insert(0,str(w/'scripts/demo'));import demo
assert not demo.project_processes() and demo.port_free()
write('source-audit.json',dict(utc=datetime.now(timezone.utc).isoformat(),implementation_sha='d7b4c1742776dfae9019f3c25241f7c9c51f04ea',run_ids=[default,changed],runtime_source_unchanged_after_tests=True,stable_worktree_head=git('rev-parse','HEAD',cwd=original),stable_clean=True,tags_unchanged=tags,protected_files_identical={n:sha(w/n) for n in protected},scene_event_controller_gpu_sources_unchanged=True,simulator_changes_only_advanced_goal_metadata_and_visual_argument=True,external=external,default_goal_restored_sha256=sha(file),remaining_project_processes=[],dds_port_free=True,v01_regression_scope='Source/tag/isolation audit only; no new stable runtime, driver changes or W13 work.'))
raw=out/'raw';raw.mkdir(exist_ok=True);tools=out/'tools';tools.mkdir(exist_ok=True)
for name in ('presentation_marker_audit.py','presentation_run_report.py','package_presentation_interface.py'):shutil.copy2(Path('/tmp')/name,tools/name)
for name in ('presentation-interface-tests.log','presentation-interface-full-tests.log','presentation-interface-build.log','presentation-interface-check-goal.log','presentation-interface-cli-checks.json','presentation-default-run.log','presentation-changed-startup.log','presentation-changed-check-goal.log','presentation-changed-run.log','presentation-changed-marker-audit.log'):
 p=Path('/tmp')/name
 if p.exists():shutil.copy2(p,raw/name)
index=[]
for root in [raw,*[w/'run_data/advanced'/v for v in (default,changed,*minimal)],*[w/'run_data'/v for v in (default,changed,*minimal)]]:
 for p in sorted(root.rglob('*')):
  if p.is_file():index.append(dict(path=str(p.relative_to(w)),bytes=p.stat().st_size,sha256=sha(p)))
write('raw-index.json',dict(files=index))
write('checks.json',dict(implementation_sha='d7b4c1742776dfae9019f3c25241f7c9c51f04ea',focused_tests_pass=27,full_tests_pass=214,historical_v01_only_source_assertion_deselected=1,system_colcon_packages=4,cli=json.loads((raw/'presentation-interface-cli-checks.json').read_text()),source_logs={p.name:sha(p) for p in raw.iterdir() if p.is_file()}))
print(json.dumps(dict(default=default,changed=changed,files=len(index),goal_default_restored=True,stable_preserved=True)))

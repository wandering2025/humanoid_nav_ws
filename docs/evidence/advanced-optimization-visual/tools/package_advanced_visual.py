from pathlib import Path
import json,hashlib,subprocess,ast,os,sys,shutil
from datetime import datetime,timezone
w=Path('/home/haoyang/humanoid_nav_ws_advanced');out=w/'docs/evidence/advanced-optimization-visual'
ident='20261016T090707Z_advanced_d2063f25-32cf-473b-bd06-ff6f2b578c7e';run=w/'run_data/advanced'/ident
video=out/'raw/recording-03';r=json.loads((run/'final-result.json').read_text())
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def git(*args,cwd=w):return subprocess.check_output(['git',*args],cwd=cwd,text=True).strip()
def write(name,value):(out/name).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
assert r['status']=='SUCCESS' and r['dynamic']['verdict']=='PASS'
assert r['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0) and not r['fell']
assert r['safe_stop']['physical_stationary'] and r['dds_port_released'] and not r['remaining_project_processes'] and not r['remaining_owned_pids']
assert all(v==0 for v in r['parent_exit_codes'].values()) and all(v['exit_code']==0 for v in r['launch_process_exit_codes'].values())
v=json.loads((video/'video-verification.json').read_text());review=json.loads((video/'review/index.json').read_text())
assert v['full_decode_verified'] and not v['visual_review_pending'] and review['status']=='PASS'
assert v['source_sha']==r['code_sha'] and sha(video/'advanced-demo.mp4')==v['sha256']==review['video_sha256']
frames=[json.loads(s) for s in (video/'frames.jsonl').read_text().splitlines()]
for f in frames:
 for image in f['images'].values():
  p=video/image['file'];assert p.stat().st_size==image['bytes'] and sha(p)==image['sha256'],p
for key in review['keyframes']:assert sha(Path(key['path']))==key['sha256']
write('final-video.json',dict(**v,visual_review=review,frame_ledger_sha256=sha(video/'frames.jsonl'),all_capture_frame_hashes_verified=True))
write('final-map-coverage.json',json.loads((run/'map-coverage.json').read_text()))
# Old evidence remains immutable, including previous failed trials and video.
old=json.loads((w/'docs/evidence/advanced-optimization/raw-index.json').read_text())
for x in old['files']:
 p=w/x['path'];assert p.stat().st_size==x['bytes'] and sha(p)==x['sha256'],p
archive=w/'run_data/gpu-recovery-archives/20261016-optimization/demo-gpu-halt.json'
assert sha(archive)=='60095afd345dff75caa25046bc7d8b81597837b13c4f8625026f3a885b66c0fc'
assert archive.read_bytes()==(w/'run_data/advanced/20261015T220015Z_advanced_c171be81-40f2-4a13-a537-2aa174bce3f5/gpu/halt.json').read_bytes()
assert not (w/'run_data/demo-gpu-halt.json').exists()
oldvideo=w/'docs/evidence/advanced/raw/final-recording-02/advanced-demo.mp4'
assert sha(oldvideo)=='3f9547f14e7ecadd4d0f115c4a562291dc4530e4de7a95d7633041d0f7a3716a'
original=Path('/home/haoyang/humanoid_nav_ws')
assert git('rev-parse','HEAD',cwd=original)=='0f51d54b1dbfdde0664966118b5546109a3f6ff2' and not git('status','--porcelain',cwd=original)
old_source=json.loads((w/'docs/evidence/advanced-optimization/source-audit.json').read_text())
protected=old_source['stable_byte_identical']
for name,digest in protected.items():
 assert sha(w/name)==digest and (w/name).read_bytes()==subprocess.check_output(['git','show','demo-v0.1.0:'+name],cwd=w)
tags={tag:dict(object=git('rev-parse',tag),commit=git('rev-parse',tag+'^{}')) for tag in ('demo-v0.1.0','demo-v0.2.0')}
assert tags==old_source['tags']
for name,ext in old_source['external'].items():
 path='/home/haoyang/robot_sim_sources/'+('booster_deploy' if name=='deploy' else 'booster_assets')
 assert git('rev-parse','HEAD',cwd=path)==ext['head'] and not git('status','--porcelain',cwd=path)
assert not git('diff','--name-only',r['code_sha'],'--','scripts','src')
base='2875228d3c27e293c80087453543be0c549f3347'
def node(commit,path,name,cls=None):
 tree=ast.parse(subprocess.check_output(['git','show',commit+':'+path],cwd=w,text=True))
 scope=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==cls).body if cls else tree.body
 return ast.dump(next(n for n in scope if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) and n.name==name),include_attributes=False)
for name in ('threshold_failure','sample','renderer_verified','verify_gl'):
 assert node(base,'scripts/demo/gpu_watch.py',name)==node(r['code_sha'],'scripts/demo/gpu_watch.py',name)
for name in ('check','halt','close'):
 assert node(base,'scripts/demo/gpu_watch.py',name,'Watch')==node(r['code_sha'],'scripts/demo/gpu_watch.py',name,'Watch')
summary=json.loads((out/'final-run.json').read_text());assert summary['navigation_and_safety_identical_to_headless_implementation']
os.environ.update(T1_DEMO_DIR='/tmp/advanced-final-visual-audit',T1_DEMO_RUN_ID='final-visual-audit')
sys.path.insert(0,str(w/'scripts/demo'));import demo
assert not demo.project_processes() and demo.port_free()
write('source-audit.json',dict(utc=datetime.now(timezone.utc).isoformat(),tested_source_sha=r['code_sha'],implementation_sha=git('rev-parse','405ba35'),functional_navigation_sha=base,current_head=git('rev-parse','HEAD'),runtime_sources_match_tested=True,navigation_safety_and_controller_match_headless_tests=True,runtime_gpu_fault_limits_ast_identical=True,advanced_startup_utilization_pct=60,default_startup_utilization_pct=40,old_raw_files_verified=len(old['files']),old_recording_preserved=True,old_halt_preserved_at=str(archive),old_halt_sha256=sha(archive),no_current_halt=True,stable_original_head=git('rev-parse','HEAD',cwd=original),stable_original_clean=True,protected_files=protected,tags=tags,external=old_source['external'],remaining_project_processes=[],dds_port_bind_pass=True,regression_scope='This continuation verifies stable source/tag/isolation. Prior complete stable U01 runtime remains historical, no new v0.1 runtime or W13 study.'))
topics=json.loads(Path('/tmp/advanced-opt-visual-topics03.json').read_text())
assert topics['run_id']==ident and topics['source_sha']==r['code_sha']
assert topics['overview_messages']>=1 and topics['matched_planner_display_hashes']>=3 and topics['overview_geometry_never_displayed_as_execution']
correspondence=json.loads((out/'live-path-correspondence.json').read_text())
assert correspondence['run_id']==ident and correspondence['all_display_geometry_and_frame_match_planner'] and correspondence['overview_never_displayed_as_execution']
write('live-path-topics.json',topics)
raw=out/'raw/analysis';raw.mkdir(exist_ok=True,parents=True);tool_dir=out/'tools';tool_dir.mkdir(exist_ok=True)
for name in ('advanced_visual_report.py','advanced_visual_review.py','advanced_visual_topic_audit.py','advanced_cdr_roundtrip.py','interpret_advanced_topics.py','package_advanced_visual.py'):
 shutil.copy2(Path('/tmp')/name,tool_dir/name)
for name in ('advanced-opt-visual-minimal01.log','advanced-opt-visual-rehearsal01.log','advanced-opt-visual-recording02.log','advanced-opt-visual-capture02.log','advanced-opt-visual-recording03.log','advanced-opt-visual-capture03.log','advanced-opt-visual-encode03.log','advanced-opt-visual-topics03.log','advanced-opt-gpu-admission-tests.log','advanced-opt-visual-contract-tests.log','advanced-opt-path-display-tests.log'):
 if (Path('/tmp')/name).exists():shutil.copy2(Path('/tmp')/name,raw/name)
attempts=[];index=[];roots=[]
for p in sorted((w/'run_data/advanced').glob('20261016*advanced*')):
 roots.extend([p,w/'run_data'/p.name]);f=p/'final-result.json'
 if f.exists():
  x=json.loads(f.read_text());attempts.append(dict(run_id=p.name,code_sha=x.get('code_sha'),status=x['status'],error=x.get('error'),result_sha256=sha(f)))
 else:attempts.append(dict(run_id=p.name,status='FAILED_STARTUP_ADMISSION',renderer_started=False,source_sha=json.loads((p/'provenance.json').read_text())['code_sha']))
roots.extend([out/'raw',archive.parent])
for root in roots:
 if not root.exists():continue
 for p in sorted(root.rglob('*')):
  if not p.is_file():continue
  if p.parent==video and p.suffix=='.png':continue # bound by verified frame ledger
  index.append(dict(path=str(p.relative_to(w)),bytes=p.stat().st_size,sha256=sha(p)))
write('attempts.json',attempts)
write('raw-index.json',dict(files=index,video_frame_ledger=dict(path=str((video/'frames.jsonl').relative_to(w)),sha256=sha(video/'frames.jsonl'),pairs=len(frames),verified_images=2*len(frames)),scope='All new full/minimal/failed-startup runs and analysis/recording metadata; real PNG frame images are individually verified against their ledger; old1177 artifacts separately rechecked.'))
print(json.dumps(dict(run_id=ident,status=r['status'],video_sha256=v['sha256'],seconds=v['total_seconds'],capture_pairs=len(frames),files=len(index),old_files_verified=len(old['files'])),indent=2))

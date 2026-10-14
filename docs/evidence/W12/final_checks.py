"""Read-only final audit of W12 recorded artifacts; run after suite + regression."""
import hashlib
import ast
import json
from pathlib import Path
import subprocess
import yaml

WS = Path(__file__).resolve().parents[3]
OUT = WS/'docs/evidence/W12'
SOURCE = '5149679a5ae1209991e0c4f62031dc352f59281f'
RUNTIME = [p for p in subprocess.check_output(['git','-C',str(WS),'ls-files','src'],text=True).splitlines() if '/maps/' not in p] + [ 'docs/evidence/W12/harness.py','docs/evidence/W12/dynamic_suite.py']
def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def git(*args):return subprocess.check_output(['git','-C',str(WS),*args],text=True).strip()
def read(name):return json.loads((OUT/name).read_text())

def evaluator_ast(source):
    tree=ast.parse(source)
    tree.body=[n for n in tree.body if not (isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='CASES' for t in n.targets))]
    return ast.dump(tree,include_attributes=False)

def evaluator_equal(ref):
    before=git('show',ref+':docs/evidence/W12/nav_run.py')
    return evaluator_ast(before)==evaluator_ast((OUT/'nav_run.py').read_text())

start = read('start-state.json')
quality = read('map-quality.json')
loc = read('localization.json')
suite = read('smoke-suite.json')
rows = [read(r['label']+'.json') for r in suite['results']]
reg = read('w12_S03.json')
wait_reg = read('w12_static_wait_stage.json')
default_reg = read('w12_S04.json')
meta_path = WS/'src/t1_navigation/maps/dynamic_crossing_world.meta.yaml'
meta = yaml.safe_load(meta_path.read_text())
map_path = meta_path.with_suffix('').with_suffix('.yaml')
config = yaml.safe_load(map_path.read_text())
events_path = WS/'src/t1_bringup/config/w12_events.yaml'
counts = {k:dict(runs=sum(r['dynamic']['scenario']==k for r in rows),
                 success=sum(r['dynamic']['scenario']==k and r['outcome']=='SUCCESS' for r in rows))
          for k in ('crossing','blocking','approach')}
checks = dict(
    user_edits_preserved=digest(WS/'docs/start_commands.md')==start['user_file_sha256'],
    static_world_unchanged=all(digest(WS/p)==v for p,v in start['frozen_sha256'].items()),
    w11_evidence_unchanged=not git('diff',start['head'],'--','docs/evidence/W11'),
    map_quality=quality['verdict']=='PASS' and all(quality['checks'].values()),
    mapping_clean=quality['generation_dirty_files']==['M docs/start_commands.md'],
    scene_bound=digest(WS/'src/t1_mujoco/worlds/dynamic_crossing_world.yaml')==meta['scene_yaml_sha256'],
    map_bound=digest(map_path)==meta['map_yaml_sha256'] and digest(map_path.parent/config['image'])==meta['map_image_sha256'],
    independent_amcl=len(loc['trials'])==3 and loc['verdict']=='PASS' and all(r['verdict']=='PASS' for r in loc['trials']),
    frozen_w09_limits=meta['expected_localization']==dict(initial_inputs=[[.2,-.15,.15],[-.2,.15,-.2],[.1,.1,.3]],
        convergence_sim_s=25,position_error_max_m=.25,yaw_error_max_rad=0.17453292519943295,scan_distance_p95_max_m=.15),
    live_event_lifecycle=read('c1-event-lifecycle.json')['verdict']=='PASS',
    scene_dimensions_and_rays=read('c1-scene.json')['verdict']=='PASS',
    three_runs_per_class=all(c['runs']>=3 for c in counts.values()),
    complete_success_per_class=all(c['success']>=1 for c in counts.values()),
    exactly_nine_runs=len(rows)==9,
    sensor_loop=all(all(v for k,v in r['dynamic']['checks'].items() if k != 'navigation_success') for r in rows),
    graph_tf_parameters=all(all(r['metrics']['graph_checks'].values()) and all(r['parameter_checks'].values()) for r in rows+[reg]),
    complete_contact_evidence=all(set(r['metrics']['env_contacts'])=={'static_env','obstacle','ground_nonfoot'} for r in rows+[reg]),
    zero_contacts_and_falls=all(not any(r['metrics']['env_contacts'].values()) and not r['metrics']['fell'] for r in rows+[reg]),
    clean_launch_and_sim=all(r['run']['exit_code']==0 and r['run']['manifest'].get('exit_code')==0 for r in rows+[reg]),
    no_subprocess_shutdown_errors=all('process has died' not in (OUT/'raw'/(r['run']['run_id']+'.txt')).read_text() for r in rows+[reg]),
    event_file_manifest=all(r['run']['manifest']['parameter_files']['events']['path']==str(events_path)
                            and r['run']['manifest']['parameter_files']['events']['sha256']==digest(events_path) for r in rows),
    safety_nav_contracts_unchanged=not git('diff',start['head'],'--',
        'src/t1_navigation/config/safety.yaml','src/t1_navigation/config/navigation.yaml',
        'src/t1_mujoco/config/command_map.yaml','src/t1_bringup/t1_bringup/safety.py',
        'src/t1_bringup/t1_bringup/navigation_adapter.py','src/t1_bringup/t1_bringup/navigation_tools.py'),
    actual_seeds_recorded=all(json.loads((WS/'run_data'/r['run']['run_id']/'manifest.json').read_text())['resolved_parameters']['seed']
                            ==r['run']['manifest']['parameter_files']['events']['seed'] for r in rows),
    same_map_all_runs=all(r['run']['manifest']['map_sha256']==digest(map_path) for r in rows),
    same_runtime_sources=not git('diff',SOURCE,'--',*RUNTIME) and all(
        not git('diff',SOURCE,r['run']['manifest']['code_sha'],'--',*RUNTIME) for r in rows+[reg]),
    static_registry_correction_only=evaluator_equal(SOURCE) and all(evaluator_equal(r['run']['manifest']['code_sha']) for r in rows+[reg]),
    static_regression=reg['outcome']=='SUCCESS',
    default_static_failure_preserved=default_reg['outcome']=='FAIL_TIMEOUT' and not any(default_reg['metrics']['env_contacts'].values()) and not default_reg['metrics']['fell'],
)
upstream={}
for name,path,expected in [('booster_deploy','/home/haoyang/robot_sim_sources/booster_deploy','7bb1462e2ac742eba5c01659b815e2e99fbe617f'),
                           ('booster_assets','/home/haoyang/robot_sim_sources/booster_assets','12a97516f57469078707afb587a5f84c1cb110d1')]:
    head=subprocess.check_output(['git','-C',path,'rev-parse','HEAD'],text=True).strip()
    dirty=subprocess.check_output(['git','-C',path,'status','--porcelain'],text=True).strip()
    upstream[name]=dict(head=head,dirty=dirty)
    checks[name+'_frozen_clean']=head==expected and not dirty
checks={k:bool(v) for k,v in checks.items()}
result=dict(verdict='PASS' if all(checks.values()) else 'FAIL',checks=checks,counts=counts,
            source_base_sha=SOURCE,run_shas=sorted(set(r['run']['manifest']['code_sha'] for r in rows+[reg])),
            static_regression=dict(S04_default=default_reg['outcome'],S04_completed_approach=wait_reg['outcome'],S03_default=reg['outcome']),
            all_nine_success=all(r['outcome']=='SUCCESS' for r in rows), failed_cases=[r['label'] for r in rows if r['outcome']!='SUCCESS'],
            upstream=upstream,not_claimed=['W13 reliability','target tracking/prediction','hardware safety certification'])
(OUT/'final-verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['verdict']=='PASS' else 1)

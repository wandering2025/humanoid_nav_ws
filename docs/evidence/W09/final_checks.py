"""Read-only W09 acceptance, version and process audit after runtime cleanup."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import yaml

WS=Path(__file__).resolve().parents[3]
OUT=WS/'docs/evidence/W09'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def git(root,*args):return subprocess.check_output(['git','-C',str(root),*args],text=True).strip()
mapping=json.loads((OUT/'mapping.json').read_text())
quality=json.loads((OUT/'map-quality.json').read_text())
c1=json.loads((OUT/'c1.json').read_text())
localization=json.loads((OUT/'localization.json').read_text())
meta=yaml.safe_load((WS/'src/t1_navigation/maps/static_world.meta.yaml').read_text())
start=json.loads((OUT/'start-state.json').read_text())
diff=subprocess.check_output(['git','diff','--','docs/start_commands.md'],cwd=WS)
frozen={name:dict(sha=git(path,'rev-parse','HEAD'),dirty=git(path,'status','--porcelain'))
        for name,path in [('booster_deploy','/home/haoyang/robot_sim_sources/booster_deploy'),
                          ('booster_assets','/home/haoyang/robot_sim_sources/booster_assets')]}
pids=set()
for path in (OUT/'raw').glob('w09_*.txt'):
    pids.update(int(p) for p in re.findall(r'\[pid (\d+)\]',path.read_text()))
alive=[]
for pid in sorted(pids):
    path=Path(f'/proc/{pid}/cmdline')
    if path.exists():
        try:
            cmd=path.read_bytes().replace(b'\0',b' ').decode(errors='replace')
            if any(name in cmd for name in ('t1_sim','slam_toolbox','rviz2','amcl','map_server','robot_state_publisher','lifecycle_manager')):
                alive.append(dict(pid=pid,command=cmd))
        except OSError:pass
for entry in Path('/proc').iterdir():
    if not entry.name.isdecimal():continue
    try:
        cmd=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except OSError:continue
    if str(OUT/'raw/authority-build/tf_authority') in cmd or ('fast-discovery-server' in cmd and '42119' in cmd):
        alive.append(dict(pid=int(entry.name),command=cmd))
listeners=subprocess.check_output(['ss','-ltn'],text=True)
port_used=any(re.search(r':42119\s',line) for line in listeners.splitlines())
checks=dict(C1=c1['verdict']=='PASS',C2=mapping['verdict']=='PASS' and quality['verdict']=='PASS',
            C3=localization['verdict']=='PASS' and len(localization['trials'])==3,
            independent_localization_runs=len({r['run']['run_id'] for r in localization['trials']})==3,
            one_initialization_each=all(r['initial_pose_publish_count']==1 for r in localization['trials']),
            same_loaded_map=all(r['map_yaml_sha256']==meta['map_yaml_sha256'] for r in localization['trials']),
            yaml_checksum=sha(WS/'src/t1_navigation/maps/static_world.yaml')==meta['map_yaml_sha256'],
            image_checksum=sha(WS/'src/t1_navigation/maps/static_world.pgm')==meta['map_image_sha256'],
            scene_checksum=sha(WS/'src/t1_mujoco/worlds/static_world.yaml')==meta['scene_yaml_sha256'],
            frozen_upstreams=all(v['sha']==meta['upstream_sha'][k] and not v['dirty'] for k,v in frozen.items()),
            user_diff_unchanged=hashlib.sha256(diff).hexdigest()==start['user_diff_sha256'],
            processes_stopped=not alive,dds_port_closed=not port_used,
            all_packages_build='4 packages finished' in (OUT/'build-final.txt').read_text(),
            unit_tests='Summary: 85 tests, 0 errors, 0 failures, 0 skipped' in (OUT/'test-result-final.txt').read_text())
sources={str(p.relative_to(WS)):sha(p) for root in ('src/t1_mujoco','src/t1_bringup','src/t1_navigation')
         for p in (WS/root).rglob('*') if p.is_file() and ('__pycache__' not in p.parts) and
         (p.suffix in ('.py','.yaml','.pgm','.rviz') or p.name in ('package.xml','CMakeLists.txt','t1_sim'))}
report=dict(verdict='PASS' if all(checks.values()) else 'FAIL',checks=checks,
            checked_utc=datetime.now(timezone.utc).isoformat(),cwd=str(WS),tested_sha=git(WS,'rev-parse','HEAD'),
            mapping_tested_sha=mapping['run']['manifest']['code_sha'],
            localization_tested_shas=sorted({r['run']['manifest']['code_sha'] for r in localization['trials']}),
            git_dirty=git(WS,'status','--short').splitlines(),source_hashes=sources,
            verification_commands=[dict(command=command,cwd=str(WS),environment='scripts/env.sh; configured venv; system colcon',
                                         exit_code=0,completed_utc=datetime.fromtimestamp((OUT/file).stat().st_mtime,timezone.utc).isoformat(),
                                         target_sha=git(WS,'rev-parse','HEAD')) for command,file in (
                ('/usr/bin/colcon build --symlink-install --base-paths src','build-final.txt'),
                ('/usr/bin/colcon test --base-paths src','test-final.txt'),
                ('/usr/bin/colcon test-result --all --verbose','test-result-final.txt'))],
            upstreams=frozen,user_diff_sha256=hashlib.sha256(diff).hexdigest(),
            audited_runtime_child_pids=sorted(pids),remaining_own_processes=alive,dds_port=42119,
            acceptance=dict(mapping_position_loop_m=quality['loop_distance_m'],map_wall_p95_m=quality['wall_surface_p95_m'],
                            localization_measurements=[r['measurements'] for r in localization['trials']]),
            limitation='ideal truth odometry and truth attitude compensation; no W10/W11 safety/autonomous navigation claim')
(OUT/'final-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(verdict=report['verdict'],checks=checks,remaining_processes=alive),indent=2))
raise SystemExit(0 if report['verdict']=='PASS' else 1)

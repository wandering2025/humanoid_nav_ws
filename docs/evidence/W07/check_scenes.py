"""C2 model/physics equivalence and configuration-driven load checks."""
from datetime import datetime, timezone
import json, os, subprocess, time
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
import numpy as np
from t1_mujoco.runtime import load_task_config
from t1_mujoco.scenes import SCENES, assemble
from t1_mujoco.model_contract import joint_map
ws=Path(os.environ['T1_WS']);assets=Path('/home/haoyang/robot_sim_sources/booster_assets')
started=datetime.now(timezone.utc).isoformat();wall=time.monotonic()
cfg=load_task_config(os.environ['T1_UPSTREAM'])
root=ET.parse(assets/'robots/T1/T1_23dof.xml').getroot()
root.find('compiler').set('meshdir',str(assets/'robots/T1/meshes'))
for i, geom in enumerate(root.iter('geom')):
    if not geom.get('name'):
        geom.set('name',f't1_geom_{i:03d}')
base=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
fields=('jnt_type','jnt_bodyid','jnt_qposadr','jnt_dofadr','jnt_pos','jnt_axis','jnt_range','dof_armature','dof_damping','actuator_trnid','actuator_gear','actuator_gainprm','actuator_biasprm','body_pos','body_quat','body_mass','body_inertia','geom_type','geom_bodyid','geom_size','geom_pos','geom_quat','geom_contype','geom_conaffinity','geom_friction','geom_solref','geom_solimp','geom_group')
results=[]
for name in SCENES:
    path, provenance=assemble(ws/'src/t1_mujoco/worlds'/f'{name}.yaml',assets,ws/'run_data/cache/scenes')
    m=mujoco.MjModel.from_xml_path(str(path));indices=joint_map(m,cfg.robot.joint_names)
    # MuJoCo groups world geoms before body geoms; added world geoms shift IDs.
    # Compare existing geom properties by preserved names, not prefix indices.
    geom_ids=[m.geom(base.geom(i).name).id for i in range(base.ngeom)]
    unchanged={key:bool(np.array_equal(getattr(base,key),getattr(m,key)[geom_ids] if key.startswith('geom_') else getattr(m,key))) for key in fields}
    unique={tag:len(names)==len(set(names)) and all(names) for tag,names in (
        ('body',[m.body(i).name for i in range(m.nbody)]),
        ('geom',[m.geom(i).name for i in range(m.ngeom)]))}
    added=[i for i in range(m.ngeom) if m.geom(i).name.startswith('env_')]
    checks=dict(physics_arrays_identical=all(unchanged.values()),dimensions=(m.nq,m.nv,m.nu,m.njnt)==(30,29,23,24),
                unique_names=all(unique.values()),env_visual_collision=all(m.geom_group[i]==2 and m.geom_contype[i]==1 and m.geom_conaffinity[i]==1 and m.geom_rgba[i,3]==1 for i in added),
                no_moving_obstacles=m.nmocap==0,gravity_identical=np.array_equal(m.opt.gravity,base.opt.gravity))
    results.append(dict(name=name,path=str(path),provenance=provenance,checks=checks,unchanged=unchanged,indices=indices.tolist(),ngeom=m.ngeom))
result=dict(code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),started_utc=started,ended_utc=datetime.now(timezone.utc).isoformat(),wall_s=time.monotonic()-wall,cwd=str(ws),command='source scripts/env.sh; $T1_VENV/bin/python docs/evidence/W07/check_scenes.py',scenes=results,verdict='PASS' if all(all(r['checks'].values()) for r in results) else 'FAIL')
(ws/'docs/evidence/W07/scene-load.json').write_text(json.dumps(result,indent=2)+'\n')
print([(r['name'],r['checks']) for r in results]);assert result['verdict']=='PASS'

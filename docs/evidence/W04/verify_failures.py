"""Installed startup rejection cases: fail explicitly, no silent stand-in."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
ws=Path(os.environ['T1_WS']);out=ws/'docs/evidence/W04'
bad_scene=ws/'run_data/w04_incompatible.xml'
bad_scene.write_text('<mujoco><worldbody><body><freejoint/><geom type="sphere" size="0.1"/></body></worldbody></mujoco>')
results=[]
for name,params,expected in (
    ('model',{'scene':str(bad_scene)},'expected nq/nv/nu/njnt'),
    ('sim_time',{'use_sim_time':'true'},'use_sim_time=false'),
    ('mode',{'mode':'navigation'},'sim_only only'),
    ('timeout',{'cmd_timeout_wall':'-1.0'},'must be finite and nonnegative'),
):
    run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_dev_reject_'+name+'_'+uuid.uuid4().hex[:6]
    cmd=[str(ws/'install/t1_mujoco/lib/t1_mujoco/t1_sim'),'--ros-args','-p','run_id:='+run_id]
    for key,value in params.items():cmd+=['-p',key+':='+value]
    began=datetime.now(timezone.utc).isoformat();wall=time.monotonic()
    p=subprocess.run(cmd,stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=15,cwd='/tmp')
    assert p.returncode==2 and expected in p.stderr,(name,p.returncode,p.stdout,p.stderr)
    results.append(dict(case=name,run_id=run_id,command=cmd,started_utc=began,
                        wall_duration=time.monotonic()-wall,sim_duration=0.,exit_code=p.returncode,stderr=p.stderr))
result=dict(code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),results=results,verdict='PASS')
(out/'startup-failures.json').write_text(json.dumps(result,indent=2)+'\n')
print('PASS: 4 explicit startup rejection cases')

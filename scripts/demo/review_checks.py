"""W15 serial CPU-only independent checks; reuses frozen test cases/predicates.

All new outputs live under W15/raw. Never overwrites W10/W13 evidence.
"""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

WS=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser();p.add_argument('case',choices=('S01','B01','source_timeout'));a=p.parse_args()
    from cpu_guard import enter
    isolation=enter()
    run_id=f'{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_w15_{a.case}_{uuid.uuid4().hex[:6]}'
    out=WS/'docs/evidence/W15/raw'/run_id;out.mkdir(parents=True)
    (out/'isolation.json').write_text(json.dumps(isolation,indent=2)+'\n')
    os.environ['ROS_LOG_DIR']=str(out/'ros-log')
    print('W15_REVIEW',run_id,flush=True)
    if a.case=='source_timeout':
        spec=importlib.util.spec_from_file_location('w10_checks',WS/'docs/evidence/W10/run_checks.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        module.OUT=out;module.RAW=out/'raw';module.RAW.mkdir()
        shutil.copyfile(WS/'docs/evidence/W10/open.yaml',out/'open.yaml')
        shutil.copyfile(WS/'docs/evidence/W10/fault.launch.py',out/'fault.launch.py')
        sys.argv=[sys.argv[0],'source_timeout']
        code=module.main()
    else:
        import yaml
        suite=yaml.safe_load((WS/'src/t1_bringup/config/test_suite_v1.yaml').read_text())
        case=next(c for c in suite['cases'] if c['id']==a.case)
        os.environ.update(T1_EVAL_OUT=str(out),T1_EVAL_RUN_ID=run_id,T1_EVAL_SCENE=case['scene'],
            T1_EVAL_EVENTS=str(WS/suite['events_file']),T1_EVAL_KIND=case.get('kind',''),T1_EVAL_HEADLESS='true')
        if case.get('scenario'):os.environ['T1_EVAL_SCENARIO']=case['scenario']
        sys.path.insert(0,str(WS/'scripts'))
        from evaluation.worker import run
        run(case,suite,out,run_id,gui=False)
        result=json.loads((out/'result.json').read_text())
        code=0 if result['outcome']=='SUCCESS' else 1
    (out/'review-exit.json').write_text(json.dumps(dict(exit_code=code,case=a.case,run_id=run_id,
        code_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip()),indent=2)+'\n')
    return code


if __name__=='__main__':raise SystemExit(main())

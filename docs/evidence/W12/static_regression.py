"""S04 targeted terminal-stage hypothesis, one existing goal-tool option changed."""
import json
import os
os.environ['W12_SCENE']='static_world'
import yaml
from harness import Session, WS, OUT
from nav_run import episode

cases=yaml.safe_load((WS/'src/t1_navigation/config/w11_static_cases.yaml').read_text())
case=next(c for c in cases['cases'] if c['id']=='S04')
session=Session('static_wait_stage')
try:
    r=episode('w12_static_wait_stage',case['start'],case['goal'],cases['criteria'],120.,
              case_id='S04_wait_stage',seed=case['seed'],rviz=True,screenshot=18.,
              tool_args=['--approach-pass-radius','0.0'])
finally:
    session.close()
raise SystemExit(0 if r['outcome']=='SUCCESS' else 1)

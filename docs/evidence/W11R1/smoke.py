"""W11R1 narrow runner; reuses the unchanged W13 evaluator and frozen cases.

Usage: configured venv smoke.py CAMPAIGN C01,C09,B04,B06
No formal-v1 writes. Only requested cases are run, once per campaign.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

WS = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WS/'scripts'))
spec = importlib.util.spec_from_file_location('w11r1_evaluate', WS/'scripts/evaluate.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.BASE = WS/'docs/evidence/W11R1/raw'
if __name__ == '__main__':
    campaign, cases = sys.argv[1:]
    if not set(cases.split(',')) <= {'C01', 'C09', 'B04', 'B06'}:
        raise SystemExit('W11R1 scope: C01,C09,B04,B06 only')
    raise SystemExit(module.run(SimpleNamespace(campaign=campaign, cases=cases, limit=0, gui=False)))

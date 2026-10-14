"""W12 development/formal cases: three starts per class, no manual commands."""
import argparse
import json
import os
import sys
import yaml

from harness import OUT, WS, Session, digest, git
from nav_run import episode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['crossing','blocking','approach','all'], default='all')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--tag', default='smoke')
    parser.add_argument('--rviz', action='store_true')
    opts = parser.parse_args()
    config = WS/'src/t1_bringup/config/w12_events.yaml'
    scenarios = yaml.safe_load(config.read_text())['scenarios']
    selected = list(scenarios) if opts.scenario == 'all' else [opts.scenario]
    session = Session('dynamic_'+opts.tag)
    results = []
    try:
        for name in selected:
            os.environ['W12_SCENARIO'] = name
            for index in range(opts.repeats):
                label = f'w12_{opts.tag}_{name}_{index+1}'
                result = episode(label, [0.,0.,0.], [9.,0.,0.],
                                 dict(position_m=.30,yaw_deg=20.,settle_s=2.), 120.,
                                 seed=scenarios[name]['seed'], rviz=opts.rviz and index==0, screenshot=12.)
                results.append(dict(label=label, outcome=result['outcome'], dynamic=result.get('dynamic'),
                                    metrics=result['metrics'], run_id=result['run']['run_id']))
                report = dict(code_sha=git('rev-parse','HEAD'), events_sha256=digest(config), results=results,
                              expected_runs=len(selected)*opts.repeats,
                              verdict='PASS' if len(results)==len(selected)*opts.repeats and all(r['outcome']=='SUCCESS' for r in results) else 'FAIL')
                (OUT/f'{opts.tag}-suite.json').write_text(json.dumps(report,indent=2)+'\n')
                print('DYNAMIC',label,result['outcome'],result.get('dynamic',{}).get('checks'),flush=True)
    finally:
        session.close()
        os.environ.pop('W12_SCENARIO',None)
    return 0 if report['verdict']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

"""Append-only assessment of the two existing authorized GUI rehearsals.

No subprocesses, GPU queries, ROS or simulation. Original results stay byte-identical.
The historical lifecycle proof is inferred from the frozen runner's SIGINT logging
contract, not fabricated as a contemporaneous measurement.
"""
import hashlib
import json
from pathlib import Path
from dds_teardown import assess
from finalize import graphics_proof

ROOT=Path(__file__).resolve().parents[2]
EVIDENCE=ROOT/'docs/evidence/W14/rviz-lifecycle'
TESTED='a963518410ad0faefad0c3b2230d367780c91212'


def record(path):
    data=path.read_bytes()
    return dict(path=str(path.relative_to(ROOT)),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())


def build():
    rows=[]
    for number in ('01','02'):
        original=EVIDENCE/f'full-{number}-final-result.json'
        raw=json.loads(original.read_text())
        assert raw['code_sha']==TESTED and raw['scenario']=='U01' and raw['seed']==4000
        directory=ROOT/'run_data/demo'/raw['run_id']
        assert (directory/'final-result.json').read_bytes()==original.read_bytes()
        audit=EVIDENCE/('full-01-between-round-audit.json' if number=='01' else 'full-02-final-host-audit.json')
        host=json.loads(audit.read_text())
        assert host['remaining_owned_pids']==[]
        assert (raw['remaining_project_processes'] if number=='01' else host['project_processes'])==[]
        assert all(host['ports'][str(port)]=='bindable' for port in (42127,42128))
        assert host['windows']['Events']==[] and host['wslg_errors']==[]
        signal_rows=[r for r in raw['shutdown_signals'] if r[0]=='dds']
        assert len(signal_rows)==1 and signal_rows[0][-1]=='SIGINT'
        # a9635... cleanup logs this signal only after poll() is None and actual
        # send_signal(SIGINT); execute verified navigation and safe_stop first.
        raw['dds_shutdown']=dict(pid=signal_rows[0][1],signal='SIGINT',alive_before_signal=True,
            ros_communication_healthy=not raw.get('error') and bool(raw['nav2']['verified']),
            navigation_and_stop_complete=bool(raw['nav2']['verified']) and
                raw['safe_stop']['ack'] is True and raw['safe_stop']['physical_stationary'] is True)
        log=directory/'dds.log';text=log.read_text()
        assert '### Server is running ###' in text and '### Server shut down ###' in text
        proof=dict(dds_log_saved=True,dds_log_sha256=record(log)['sha256'],
            dds_returncode=raw['parent_exit_codes']['dds'],dds_pid_gone=True,dds_port_released=True,
            remaining_owned_pids=[],remaining_project_processes=[],
            host_graphics_checked_without_fault=graphics_proof(directory),runner_exit_code=raw['runner_exit_code'])
        classification=assess(raw,proof)
        if number=='01':
            assert raw['status']=='SUCCESS' and raw['validation_before_port_check'] is True
            assert all(value==0 for value in raw['parent_exit_codes'].values())
            # Normal zero DDS exit is not an exception. Reuse all other gates.
            assert classification['failed_checks']==['exact_nonzero_dds_code_saved','runner_finished_normally']
            classification.update(status='SUCCESS',exception=None,failed_checks=[])
            classification['checks'].pop('exact_nonzero_dds_code_saved')
            classification['checks'].pop('runner_finished_normally')
        else:
            assert raw['status']=='FAILED' and proof['dds_returncode']==-11
            assert classification['status']=='SUCCESS_WITH_WARNING',classification
        rows.append(dict(run_id=raw['run_id'],code_sha=TESTED,original_status=raw['status'],
            publication_status=classification['status'],navigation='NAVIGATION_SUCCESS',safety='SAFETY_PASS',
            warning='DDS_TEARDOWN_WARNING' if number=='02' else None,dds_returncode=proof['dds_returncode'],
            classification=classification,proof=proof,
            lifecycle_proof_method='Retrospective inference from frozen runner poll/send/log order and saved result; no new run',
            sources=[record(original),record(directory/'result.json'),record(log),record(audit),
                record(directory/'gpu/host/windows-checks.json')]))
    return dict(status='W14_DEMO_READY_WITH_DDS_TEARDOWN_WARNING',
        decision_sha='f687f21922245aa6cace42a9cef204c18f50f9b2',video_status='FINAL_VIDEO_PENDING_W15',
        new_gpu_or_gui_runs=0,rehearsals=rows,
        preserved_limitations=['W13 FAILED','original black-screen','CPU dynamic-stop failure','RViz -6',
            'original second GUI FAILED / DDS -11','dxg adapter-query ioctl errors not proven harmless'])


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    result=build()
    with args.output.open('x') as output:output.write(json.dumps(result,indent=2)+'\n')
    for row in result['rehearsals']:
        if row['warning']:print(f"DDS_TEARDOWN_WARNING dds_returncode={row['dds_returncode']} run_id={row['run_id']} (publication assessment; original FAILED retained)")
    print(result['status'])

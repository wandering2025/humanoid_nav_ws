"""Check ports only after the ROS runner exits. No ROS/GPU imports or signals."""
import argparse
import hashlib
import sys
from dds_teardown import assess
from gpu_watch import ERROR
import json
from pathlib import Path
import socket
import time


def remaining_owned(directory):
    result=[]
    for r in json.loads((directory/'owned-processes.json').read_text()):
        try:
            fields=Path('/proc',str(r['pid']),'stat').read_text().rsplit(')',1)[1].split()
            if fields[19]==r['start'] and fields[0]!='Z':result.append(r['pid'])
        except OSError:pass
    return result


def bindable():
    try:
        with socket.socket() as sock:sock.bind(('127.0.0.1',42127))
        return True
    except OSError:return False


def project_processes():
    """Read-only final audit, including project processes missed by child tracking."""
    found=[]
    root=str(Path(__file__).resolve().parents[2])
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
            parts=cmd.split()
            if not parts:continue
            executable=Path(parts[0]).name
            if (executable in ('t1_sim','navigation_starter','navigate_to_pose','command_gate',
                               'navigation_adapter','t1_demo_rviz')
                or ('ros2' in executable and 'launch t1_bringup' in cmd)
                or (executable=='rviz2' and root in cmd)):
                found.append(dict(pid=int(proc.name),command=cmd))
        except OSError:pass
    return found


def graphics_proof(directory):
    # Existing saved evidence only. Never issue a GPU or Windows query here.
    try:
        host=directory/'gpu/host'
        checks=json.loads((host/'windows-checks.json').read_text())
        if not checks or any(row.get('Events') != [] for row in checks):return False
        if not (host/'windows-baseline.json').is_file():return False
        for name in ('weston.log.delta','stderr.log.delta'):
            path=host/name
            if path.exists() and ERROR.search(path.read_text(errors='replace')):return False
        return True
    except (OSError,ValueError,TypeError):return False


def evidence(directory,raw,runner_exit,left,ready):
    log=directory/'dds.log'
    data=log.read_bytes() if log.is_file() else b''
    pid=raw.get('dds_shutdown',{}).get('pid')
    return dict(dds_log_saved=bool(data) and b'### Server is running ###' in data and b'### Server shut down ###' in data,dds_log_sha256=hashlib.sha256(data).hexdigest() if data else None,
        dds_returncode=raw.get('parent_exit_codes',{}).get('dds'),
        dds_pid_gone=type(pid) is int and not Path('/proc',str(pid)).exists(),
        dds_port_released=ready,remaining_owned_pids=left,remaining_project_processes=project_processes(),
        host_graphics_checked_without_fault=graphics_proof(directory),runner_exit_code=runner_exit)


def finish(directory,runner_exit,timeout=65.):
    directory=Path(directory);raw=json.loads((directory/'result.json').read_text())
    before=time.monotonic();left=remaining_owned(directory)
    ready=bindable()
    if not ready:print('[demo] ROS runner exited; waiting for bounded TCP cooldown.',flush=True)
    end=before+timeout
    while not left and not ready and time.monotonic()<end:
        time.sleep(.1);ready=bindable()
    result=dict(raw,runner_exit_code=runner_exit,post_runner_remaining_pids=left,
                dds_port_released=ready,post_runner_port_wait_wall_s=time.monotonic()-before)
    pending=raw.get('status')=='PENDING_PORT_CHECK' and raw.get('validation_before_port_check') is True and runner_exit==4
    result['status']=('SUCCESS_STARTUP_CHECK' if raw.get('startup_check_completed') else 'SUCCESS') if pending and ready and not left else 'FAILED'
    result['dds_returncode']=raw.get('parent_exit_codes',{}).get('dds')
    if result['status']=='FAILED':
        proof=evidence(directory,raw,runner_exit,left,ready)
        classification=assess(raw,proof)
        result['dds_teardown_assessment']=classification
        result['dds_teardown_evidence']=proof
        if classification['status']=='SUCCESS_WITH_WARNING':
            result['status']='SUCCESS_WITH_WARNING'
            result['navigation_classification']='NAVIGATION_SUCCESS'
            result['safety_classification']='SAFETY_PASS'
            result['terminal_warning']=f'DDS_TEARDOWN_WARNING dds_returncode={result["dds_returncode"]}'
            print(f'!!! {result["terminal_warning"]} — saved child exit/log; release exception only !!!',file=sys.stderr,flush=True)
    (directory/'final-result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(f'[demo] Final {result["status"]} | remaining={left} | port released={ready} | {directory}/final-result.json',flush=True)
    return 0 if result['status'].startswith('SUCCESS') else (runner_exit if runner_exit not in (0,4) else 1)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('runner_exit',type=int)
    args=parser.parse_args();raise SystemExit(finish(args.directory,args.runner_exit))

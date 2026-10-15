"""Check ports only after the ROS runner exits. No ROS/GPU imports or signals."""
import argparse
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
    (directory/'final-result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(f'[demo] Final {result["status"]} | remaining={left} | port released={ready} | {directory}/final-result.json',flush=True)
    return 0 if result['status'].startswith('SUCCESS') else (runner_exit if runner_exit not in (0,4) else 1)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('runner_exit',type=int)
    args=parser.parse_args();raise SystemExit(finish(args.directory,args.runner_exit))

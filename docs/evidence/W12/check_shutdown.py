"""Ten fresh ROS command-gate SIGINT exits after the teardown race fix."""
import json
import signal
import subprocess
import time
from harness import OUT, RAW, WS, Session

session=Session('shutdown_probe')
results=[]
try:
    for index in range(10):
        path=RAW/f'shutdown-gate-{index}.txt'
        with path.open('w') as output:
            proc=subprocess.Popen([str(WS/'install/t1_bringup/lib/t1_bringup/command_gate'),'--ros-args','-p','use_sim_time:=true'],
                                  stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(1.3)
            proc.send_signal(signal.SIGINT)
            code=proc.wait(timeout=15)
        text=path.read_text()
        results.append(dict(index=index,exit_code=code,traceback='Traceback' in text,
                            invalid_context='context is invalid' in text))
        print('EXIT',results[-1],flush=True)
finally:
    session.close()
report=dict(verdict='PASS' if len(results)==10 and all(r['exit_code']==0 and not r['traceback'] for r in results) else 'FAIL',trials=results)
(OUT/'shutdown-probe.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['verdict']=='PASS' else 1)

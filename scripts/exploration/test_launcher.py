"""Execute the real entrypoint's child-wait section with a controllable child."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

WS=Path(__file__).resolve().parents[2]

@pytest.mark.parametrize('exit_code,interrupts', [(0,0),(2,0),(0,1),(2,1),(0,2)])
def test_launcher_preserves_child_exit_after_interrupt(tmp_path,exit_code,interrupts):
    source=(WS/'scripts/run_exploration_demo.sh').read_text()
    block=source[source.index("runner=''\n"):source.index('if [[ -f "$T1_DEMO_DIR/result.json" ]]')]
    fake=tmp_path/'scripts/exploration/demo.py';fake.parent.mkdir(parents=True)
    fake.write_text('''import os,signal,time
ending=None
def stop(*_):
 global ending
 if ending is None: ending=time.monotonic()+.15
signal.signal(signal.SIGTERM,stop)
print('ready '+str(os.getpid()),flush=True)
'''+('ending=time.monotonic()+.02\n' if not interrupts else '')+'''
while ending is None or time.monotonic()<ending:time.sleep(.005)
raise SystemExit('''+str(exit_code)+')\n')
    child=subprocess.Popen(['bash','-c','set -Eeuo pipefail\n'+block+'\nexit "$code"\n'],
        env=dict(os.environ,EXP_ROOT=str(tmp_path),T1_VENV=sys.prefix),
        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
    owned_pid=owned_start=None
    try:
        ready=child.stdout.readline().strip();assert ready.startswith('ready '),ready
        owned_pid=int(ready.split()[1])
        owned_start=Path(f'/proc/{owned_pid}/stat').read_text().rsplit(')',1)[1].split()[19]
        for _ in range(interrupts):
            child.send_signal(signal.SIGINT);time.sleep(.025)
        output,error=child.communicate(timeout=3)
        assert child.returncode==exit_code,(child.returncode,output,error)
        with pytest.raises(ProcessLookupError):os.kill(owned_pid,0)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGTERM)
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
        if owned_pid:
            try:
                current=Path(f'/proc/{owned_pid}/stat').read_text().rsplit(')',1)[1].split()[19]
                if current==owned_start:os.kill(owned_pid,signal.SIGTERM)
            except (FileNotFoundError,ProcessLookupError):pass

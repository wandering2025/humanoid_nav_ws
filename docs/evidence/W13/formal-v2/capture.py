"""Add exact DDS return-code evidence around the byte-frozen W13 entry point.

Parent mode changes only the worker's Python bootstrap to this file. Worker mode
calls the original entry point and original Session.close exactly once. No ROS,
scenario, command, timeout, result predicate or signal-delivery changes.
"""
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import time

WS = Path('/home/haoyang/humanoid_nav_ws')
ENTRY = WS/'scripts/evaluate.py'
sys.path.insert(0, str(WS/'scripts'))

if len(sys.argv) > 1 and sys.argv[1] == 'worker':
    from evaluation import harness
    out = Path(os.environ['T1_EVAL_OUT'])
    signals = []

    def audit(event, args):
        if event in ('os.kill', 'os.killpg') and args[1] != 0:
            signals.append(dict(event=event, target=args[0], signal=args[1], wall=time.monotonic()))

    sys.addaudithook(audit)
    original_close = harness.Session.close

    def close_with_evidence(self):
        start = time.monotonic()
        before = self.server.poll()
        error = None
        try:
            return original_close(self)
        except BaseException as exc:
            error = repr(exc)
            raise
        finally:
            value = dict(run_id=os.environ['T1_EVAL_RUN_ID'], dds_pid=self.server.pid,
                         dds_returncode=self.server.returncode, dds_returncode_before_close=before,
                         close_start_wall=start, close_end_wall=time.monotonic(), close_error=error,
                         signals=signals, launch=[dict(pid=r.proc.pid, returncode=r.proc.returncode,
                            authority_pid=r.authority.pid if r.authority else None,
                            authority_returncode=r.authority.returncode if r.authority else None,
                            children=[dict(label=l, pid=p.pid, returncode=p.returncode) for l,p,_ in r.children])
                            for r in harness.ACTIVE_RUNS])
            (out/'support-exit.json').write_text(json.dumps(value,indent=2)+'\n')

    harness.Session.close = close_with_evidence
else:
    original_popen = subprocess.Popen

    def recording_worker(args, *positional, **kwargs):
        if isinstance(args, list) and len(args) == 4 and args[1:3] == [str(ENTRY), 'worker']:
            args = [args[0], str(Path(__file__).resolve()), *args[2:]]
        return original_popen(args, *positional, **kwargs)

    subprocess.Popen = recording_worker

sys.argv = [str(ENTRY), *sys.argv[1:]]
runpy.run_path(str(ENTRY), run_name='__main__')

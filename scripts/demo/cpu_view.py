"""Software-only display child; GPU devices remain denied by Landlock."""
import json
import os
from pathlib import Path
import subprocess
import sys
from cpu_guard import enter

report=enter(software_display=True)
if not os.environ.get('DISPLAY'):raise SystemExit('CPU display requires DISPLAY')
if len(sys.argv)>1 and sys.argv[1]=='--check':
    probe=subprocess.run(['glxinfo','-B'],capture_output=True,text=True,timeout=10)
    lines=[l for l in probe.stdout.splitlines() if 'OpenGL renderer string:' in l]
    if probe.returncode or not lines or not all('llvmpipe' in line.lower() for line in lines):
        raise SystemExit(f'CPU renderer verification failed: {probe.stdout}\n{probe.stderr}')
    print(json.dumps(dict(guard=report,renderer=lines[0])),flush=True)
else:
    # Only the fixed RViz executable; no general unguarded command launcher.
    os.execv('/opt/ros/jazzy/lib/rviz2/rviz2',['rviz2',*sys.argv[1:]])

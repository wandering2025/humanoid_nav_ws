"""Final W04 suite, preserving a fixed implementation SHA and per-command evidence."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ws=Path(os.environ['T1_WS']);out=ws/'docs/evidence/W04'
assert os.environ['T1_LOCAL_DDS']=='1'
assert os.environ['ROS_DISCOVERY_SERVER'].startswith('TCPv4:[127.0.0.1]:')
head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
source_files=subprocess.check_output(['git','ls-files','src','scripts'],text=True).splitlines()
hashes={name:hashlib.sha256((ws/name).read_bytes()).hexdigest() for name in source_files}
# Installed control code must match the source bytes, not merely import successfully.
import t1_mujoco
for source in (ws/'src/t1_mujoco/t1_mujoco').glob('*.py'):
    installed=Path(t1_mujoco.__file__).parent/source.name
    assert installed.read_bytes()==source.read_bytes(),str(source)
results=[];server=None
python=os.environ['T1_VENV']+'/bin/python'
commands=[
    ('runner',[python,str(out/'check_runner.py')]),
    ('controls',[python,str(out/'verify_ros.py'),'controls']),
    ('standing',[python,str(out/'verify_ros.py'),'standing']),
    ('startup',[python,str(out/'verify_failures.py')]),
    ('gui-sigterm',[python,str(out/'verify_gui.py'),'--close','sigterm']),
    ('gui-window',[python,str(out/'verify_gui.py'),'--close','window']),
    ('launch',['ros2','launch','t1_bringup','simulation.launch.py','target_rtf:=0.0','max_sim_time:=3.0']),
    ('bringup',['ros2','launch','t1_bringup','bringup.launch.py','enable_sim:=true','target_rtf:=0.0','max_sim_time:=3.0']),
]
try:
    log=(out/'raw/final-dds.txt').open('w')
    server=subprocess.Popen([str(ws/'scripts/start_local_dds.sh')],stdout=log,stderr=subprocess.STDOUT)
    time.sleep(.5)
    assert server.poll() is None,'dedicated discovery server failed to start'
    for name,cmd in commands:
        began=datetime.now(timezone.utc).isoformat();wall=time.monotonic()
        with (out/('final-'+name+'.txt')).open('w') as f:
            p=subprocess.run(cmd,cwd=ws,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,timeout=60)
        results.append(dict(name=name,command=cmd,cwd=str(ws),started_utc=began,
                            ended_utc=datetime.now(timezone.utc).isoformat(),wall_duration=time.monotonic()-wall,
                            code_sha=head,exit_code=p.returncode,log='final-'+name+'.txt'))
        print(name,p.returncode,flush=True)
        assert p.returncode==0,name
finally:
    if server is not None and server.poll() is None:
        server.send_signal(signal.SIGINT);server.wait(timeout=5)
    if server is not None:log.close()
    assert hashes=={name:hashlib.sha256((ws/name).read_bytes()).hexdigest() for name in source_files}
    (out/'final-verification.json').write_text(json.dumps(dict(code_sha=head,implementation_sha256=hashes,
        environment={key:os.environ.get(key) for key in ('T1_WS','T1_VENV','T1_UPSTREAM','T1_LOCAL_DDS','T1_DDS_PORT',
             'ROS_DISTRO','ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_DISCOVERY_SERVER','ROS_SUPER_CLIENT','ROS_LOG_DIR')},
        results=results,server_exit=server.returncode if server else None,
        verdict='PASS' if len(results)==len(commands) and all(x['exit_code']==0 for x in results) else 'FAIL'),indent=2)+'\n')

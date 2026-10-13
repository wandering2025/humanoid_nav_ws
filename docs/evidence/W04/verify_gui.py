"""GUI lifecycle independent of ROS graph discovery; screenshots actual window."""
import argparse
import ctypes
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

parser=argparse.ArgumentParser();parser.add_argument('--close',choices=['sigterm','window'],default='sigterm');args=parser.parse_args()
ws=Path(os.environ['T1_WS']);out=ws/'docs/evidence/W04'
run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_dev_gui_s0_'+uuid.uuid4().hex[:6]
path=ws/'run_data'/run_id
command=[str(ws/'install/t1_mujoco/lib/t1_mujoco/t1_sim'),'--ros-args','-p','headless:=false','-p','run_id:='+run_id]
result=dict(started_utc=datetime.now(timezone.utc).isoformat(),command=command,run_id=run_id,
            code_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip())
with (out/'raw'/f'{run_id}.txt').open('w') as log:
    p=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,cwd='/tmp')
    try:
        end=time.monotonic()+20
        while time.monotonic()<end:
            if p.poll() is not None:raise RuntimeError(f'early exit {p.returncode}')
            if (path/'events.jsonl').exists() and '"state": "READY"' in (path/'events.jsonl').read_text():break
            time.sleep(.1)
        else:raise RuntimeError('viewer failed to reach READY')
        time.sleep(1)
        tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
        windows=[line.split()[0] for line in tree.splitlines() if 'MuJoCo' in line]
        assert windows,tree
        subprocess.run(['import','-window',windows[-1],str(out/'gui-viewer.png')],check=True,timeout=10)
        if args.close=='sigterm':
            p.send_signal(signal.SIGTERM)
        else:
            # Send the window manager's standard close request to this viewer only.
            x=ctypes.CDLL('libX11.so.6')
            x.XOpenDisplay.restype=ctypes.c_void_p
            x.XInternAtom.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int]
            x.XInternAtom.restype=ctypes.c_ulong
            x.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.c_void_p]
            x.XFlush.argtypes=[ctypes.c_void_p];x.XCloseDisplay.argtypes=[ctypes.c_void_p]
            class Client(ctypes.Structure):
                _fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send_event',ctypes.c_int),
                          ('display',ctypes.c_void_p),('window',ctypes.c_ulong),('message_type',ctypes.c_ulong),
                          ('format',ctypes.c_int),('data',ctypes.c_long*5)]
            class Event(ctypes.Union):
                _fields_=[('client',Client),('pad',ctypes.c_long*24)]
            display=x.XOpenDisplay(None);assert display
            e=Event();e.client.type=33;e.client.display=display;e.client.window=int(windows[-1],16)
            e.client.message_type=x.XInternAtom(display,b'WM_PROTOCOLS',False);e.client.format=32
            e.client.data[0]=x.XInternAtom(display,b'WM_DELETE_WINDOW',False)
            assert x.XSendEvent(display,e.client.window,False,0,ctypes.byref(e))
            x.XFlush(display);x.XCloseDisplay(display)
        result['close_method']=args.close
        result['exit_code']=p.wait(timeout=10)
        result['manifest']=json.loads((path/'manifest.json').read_text())
        result['verdict']='PASS' if p.returncode==0 else 'FAIL'
    finally:
        if p.poll() is None:
            p.send_signal(signal.SIGTERM)
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait()
        result['ended_utc']=datetime.now(timezone.utc).isoformat()
        (out/('gui-'+args.close+'.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='manifest'}))
assert result['exit_code']==0

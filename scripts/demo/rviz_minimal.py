"""Exactly one authorized minimal RViz lifecycle test, never an automatic retry."""
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid
from gpu_watch import Watch,GPU_ENV,HALT,ERROR,renderer_verified

WS=Path(__file__).resolve().parents[2]
ATTEMPT=WS/'run_data/w14-rviz-lifecycle-20261015/minimal-attempt.json'


def claim(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as f:json.dump(value,f,indent=2)


def port_free():
    try:
        with socket.socket() as s:s.bind(('127.0.0.1',42128))
        return True
    except OSError:return False


def main():
    import yaml
    rid=f'{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_rviz_minimal_{uuid.uuid4().hex[:8]}'
    out=WS/'docs/evidence/W14/raw/rviz-lifecycle'/rid
    claim(ATTEMPT,dict(run_id=rid,output=str(out),authorization='one minimal RViz only'))
    out.mkdir(parents=True,exist_ok=False)
    result=dict(run_id=rid,code_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip(),started_utc=datetime.now(timezone.utc).isoformat(),status='STARTING',exit_codes={},signals=[])
    children=[];watch=None;started=time.monotonic()
    def interrupted(*_):raise InterruptedError('External interruption; no retry')
    signal.signal(signal.SIGINT,interrupted);signal.signal(signal.SIGTERM,interrupted)
    def spawn(name,cmd):
        with (out/(name+'.log')).open('w') as f:p=subprocess.Popen(cmd,env=dict(os.environ,**GPU_ENV,T1_RVIZ_HEARTBEAT=str(out/'heartbeat')),stdout=f,stderr=subprocess.STDOUT,start_new_session=True,cwd=WS)
        children.append((name,p));return p
    def check():
        watch.check([out/'rviz.log',out/'dds.log',out/'publisher.log'])
        if any(p.poll() is not None for _,p in children):raise RuntimeError('Minimal child exited unexpectedly')
        if time.monotonic()-started>75:raise TimeoutError('Minimal startup/display wall deadline')
        if (out/'heartbeat').exists() and time.time()-(out/'heartbeat').stat().st_mtime>3:raise RuntimeError('RViz event loop unresponsive3s')
    try:
        assert port_free(),'DDS42128 occupied; no cleanup of unknown owners'
        if HALT.exists():HALT.rename(out/'previous-halt.json')
        watch=Watch(out/'gpu',idle_wait_s=60)
        config=yaml.safe_load((WS/'src/t1_navigation/config/demo.rviz').read_text())
        config['Visualization Manager']['Displays']=[d for d in config['Visualization Manager']['Displays'] if d['Class']=='rviz_default_plugins/PointCloud2']
        config['Window Geometry']=dict(Width=900,Height=650)
        (out/'minimal.rviz').write_text(yaml.safe_dump(config,sort_keys=False))
        spawn('dds',[str(WS/'scripts/start_local_dds.sh')])
        end=time.monotonic()+10
        while '### Server is running ###' not in (out/'dds.log').read_text():
            check()
            if time.monotonic()>end:raise TimeoutError('DDS startup10s')
            time.sleep(.1)
        spawn('publisher',[sys.executable,str(WS/'scripts/demo/minimal_map_source.py')])
        spawn('rviz',[str(WS/'build/demo-rviz/t1_demo_rviz'),'-d',str(out/'minimal.rviz')])
        end=time.monotonic()+25
        while not (out/'heartbeat').exists():
            check()
            if time.monotonic()>end:raise TimeoutError('RViz startup25s')
            time.sleep(.1)
        assert renderer_verified((out/'rviz.log').read_text()),'Actual RViz GL renderer not NVIDIA4050'
        end=time.monotonic()+20
        while time.monotonic()<end:check();time.sleep(.1)
        result['display_verified']=True
    except BaseException as exc:
        result['error']=repr(exc)
        if watch:watch.halt(exc)
    finally:
        # Reverse startup order: Qt/display first, then publisher process, DDS last.
        for name,p in reversed(children):
            if p.poll() is None:p.send_signal(signal.SIGINT);result['signals'].append([name,p.pid,'SIGINT'])
            try:
                end=time.monotonic()+12
                while p.poll() is None and time.monotonic()<end:
                    if watch and not result.get('error'):
                        try:watch.check([out/'rviz.log',out/'dds.log',out/'publisher.log'])
                        except Exception as exc:result['error']=repr(exc);watch.halt(exc)
                    time.sleep(.1)
                if p.poll() is None:raise subprocess.TimeoutExpired(name,12)
            except subprocess.TimeoutExpired:
                result['error']=f'{name} failed normal exit; mandatory stop, no retry'
                p.terminate();result['signals'].append([name,p.pid,'SIGTERM'])
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    result['sigkill_required']=True;p.kill();p.wait(timeout=3);result['signals'].append([name,p.pid,'SIGKILL'])
            result['exit_codes'][name]=p.returncode
        if watch:
            try:watch.host.check(ERROR,windows=True)
            except Exception as exc:result['error']=repr(exc);watch.halt(exc)
            result['gpu_max']={k:max(r[k] for r in watch.samples) for k in ('temperature_c','memory_mib','utilization_pct')};watch.close()
        result['remaining_pids']=[p.pid for _,p in children if p.poll() is None]
        end=time.monotonic()+65
        while not port_free() and time.monotonic()<end:time.sleep(.1)
        result['port_released']=port_free();result['wall_total_s']=time.monotonic()-started
        log=(out/'rviz.log').read_text(errors='replace') if (out/'rviz.log').exists() else ''
        result['clean_lifecycle']='RVIZ_LIFECYCLE clean_exit' in log
        result['status']='PASS' if result.get('display_verified') and result['clean_lifecycle'] and result['exit_codes']==dict(rviz=0,publisher=0,dds=0) and not result.get('error') and not result['remaining_pids'] and result['port_released'] else 'FAIL'
        if result['status']=='FAIL' and not HALT.exists():HALT.write_text(json.dumps(dict(reason='Minimal RViz failed; authorization exhausted',output=str(out)))+'\n')
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(result=result,output=str(out))),flush=True)
    return 0 if result['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())

"""W15 bounded static presentation check. No policy, ROS or physics steps."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

WS=Path(__file__).resolve().parents[2]


def render(model_path, heartbeat):
    sys.path.insert(0,str(WS/'src/t1_mujoco'))
    import mujoco
    import mujoco.viewer
    from t1_mujoco.presentation import style_model,decorate
    m=mujoco.MjModel.from_xml_path(model_path);d=mujoco.MjData(m)
    d.qpos[:3]=[0,0,.72];d.qpos[3]=1
    if m.nmocap:d.mocap_pos[0]=[6,-2,.75]
    mujoco.mj_forward(m,d);style_model(m)
    with mujoco.viewer.launch_passive(m,d,show_left_ui=False,show_right_ui=False) as v:
        with v.lock():decorate(v)
        start=time.monotonic()
        while time.monotonic()-start<15:
            if not v.is_running():raise RuntimeError('Visual check closed early')
            v.sync(state_only=True);heartbeat.write_text(str(time.monotonic()))
            time.sleep(1/15)


def check(output, model):
    from gpu_watch import Watch,GPU_ENV,verify_gl,renderer_verified
    output.mkdir(parents=True,exist_ok=False);os.environ.update(GPU_ENV)
    watch=None;child=None;result={'status':'NOT_RUN','model':model}
    try:
        watch=Watch(output/'gpu',idle_wait_s=60);verify_gl(watch,output)
        with (output/'renderer.log').open('w') as log:
            child=subprocess.Popen([sys.executable,__file__,'--render','--model',model,'--output',str(output)],stdout=log,stderr=subprocess.STDOUT)
            result['pid']=child.pid;start=time.monotonic();captured=False
            while child.poll() is None:
                watch.check([output/'renderer.log'])
                if time.monotonic()-start>30:raise TimeoutError('Static viewer deadline')
                hb=output/'heartbeat'
                if hb.exists() and time.time()-hb.stat().st_mtime>3:raise RuntimeError('Static viewer stalled')
                if hb.exists() and not captured:
                    from windows import XWindows,find_windows
                    ids=find_windows()
                    if 'MuJoCo' in ids:
                        window=XWindows()
                        try:window.layout(ids)
                        finally:window.close()
                        subprocess.run(['import','-window',str(ids['MuJoCo']),str(output/'view.png')],check=True,timeout=5)
                        captured=True
                time.sleep(.1)
            if child.returncode:raise RuntimeError(f'Viewer exit {child.returncode}')
            if not captured:raise RuntimeError('No configured viewer screenshot captured')
            watch.host.check(__import__('gpu_watch').ERROR,windows=True)
            result.update(status='PASS',exit_code=child.returncode)
    except BaseException as exc:
        result.update(status='FAIL',error=repr(exc))
        if watch:watch.halt(exc)
        if child and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
    finally:
        if watch:result['gpu_samples']=watch.samples;watch.close()
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='gpu_samples'}),flush=True)
    return int(result['status']!='PASS')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--render',action='store_true');p.add_argument('--model',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.render:render(a.model,a.output/'heartbeat')
    else:raise SystemExit(check(a.output,a.model))

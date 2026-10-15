"""User-authorized bounded NVIDIA sampling and one-renderer stage ownership."""
import argparse
from datetime import datetime,timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

WS=Path(__file__).resolve().parents[2]
SMI='/usr/lib/wsl/lib/nvidia-smi'
HALT=WS/'run_data/demo-gpu-halt.json'
GPU_ENV=dict(GALLIUM_DRIVER='d3d12',MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA GeForce RTX 4050',
             LIBGL_ALWAYS_SOFTWARE='false',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
ERROR=re.compile(r'Removing Device|device lost|device removed|GPU hang|GPU fault|Xid|DXGI_ERROR|segmentation fault',re.I)


def sample():
    p=subprocess.run([SMI,'--query-gpu=index,name,driver_version,temperature.gpu,memory.used,memory.total,utilization.gpu,power.draw',
                      '--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=3)
    if p.returncode:
        raise RuntimeError(f'NVML query exit {p.returncode}: {p.stderr[:300]}')
    fields=[v.strip() for v in p.stdout.splitlines()[0].split(',')]
    if len(fields)!=8 or 'RTX 4050' not in fields[1]:
        raise RuntimeError(f'Unexpected NVIDIA query: {p.stdout[:300]}')
    values=dict(temperature_c=float(fields[3]),memory_mib=float(fields[4]),memory_total_mib=float(fields[5]),
                utilization_pct=float(fields[6]),power_w=float(fields[7]))
    if not all(math.isfinite(v) for v in values.values()):
        raise RuntimeError('Non-finite or unavailable GPU monitor fields')
    return dict(wall_mono=time.monotonic(),utc=datetime.now(timezone.utc).isoformat(),index=fields[0],name=fields[1],driver=fields[2],**values)


def threshold_failure(row, baseline_memory, high_since, now):
    if row['temperature_c']>=75:return 'GPU temperature >=75C',high_since
    if row['memory_mib']>=row['memory_total_mib']*.7 or row['memory_mib']-baseline_memory>=768:
        return 'GPU memory limit/growth exceeded',high_since
    high_since=(high_since if high_since is not None else now) if row['utilization_pct']>=95 else None
    if high_since is not None and now-high_since>=5:return 'GPU load >=95% for 5s',high_since
    return None,high_since


class Watch:
    def __init__(self,output):
        if HALT.exists():raise RuntimeError(f'Previous GPU halt is latched: {HALT}; do not retry automatically')
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=True)
        self.lock=(WS/'run_data/demo-gpu.lock').open('a')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.samples=[];self.baseline=None;self.high_since=None;self.next_sample=0.
        self.check([])
        if self.samples[-1]['temperature_c']>60 or self.samples[-1]['utilization_pct']>40:
            self.close();raise RuntimeError('GPU not idle/cool enough to start (<=60C, <=40% load required)')

    def check(self,logs):
        now=time.monotonic()
        if now>=self.next_sample:
            row=sample();self.samples.append(row)
            self.next_sample=now+1.
            if self.baseline is None:self.baseline=row['memory_mib']
            with (self.output/'gpu-samples.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
            failure,self.high_since=threshold_failure(row,self.baseline,self.high_since,now)
            if failure:raise RuntimeError(failure)
        for path in logs:
            if Path(path).exists():
                text=Path(path).read_text(errors='replace')
                match=ERROR.search(text)
                if match:raise RuntimeError(f'GPU error in {path}: {match.group()}')

    def halt(self,reason):
        data=dict(reason=str(reason),utc=datetime.now(timezone.utc).isoformat(),output=str(self.output))
        HALT.write_text(json.dumps(data,indent=2)+'\n')
        (self.output/'halt.json').write_text(json.dumps(data,indent=2)+'\n')

    def close(self):
        if not self.lock.closed:self.lock.close()


def run_stage(name,command,limit):
    output=WS/'docs/evidence/W14/raw/nvidia-stages'/f'{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}_{name}_{uuid.uuid4().hex[:6]}'
    output.mkdir(parents=True)
    watch=None;p=None;result=dict(stage=name,command=command,started_utc=datetime.now(timezone.utc).isoformat())
    started=time.monotonic()
    try:
        watch=Watch(output)
        env=dict(os.environ,**GPU_ENV,T1_GPU_HEARTBEAT=str(output/'heartbeat'))
        with (output/'renderer.log').open('w') as log:
            p=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            result['pid']=p.pid
            while p.poll() is None:
                watch.check([output/'renderer.log'])
                if time.monotonic()-started>limit:raise TimeoutError('Stage wall deadline')
                heartbeat=output/'heartbeat'
                if heartbeat.exists() and time.time()-heartbeat.stat().st_mtime>3:
                    raise RuntimeError('Renderer heartbeat stale for 3s')
                time.sleep(.1)
            watch.next_sample=0.
            watch.check([output/'renderer.log'])
            if p.returncode!=0:raise RuntimeError(f'Renderer exit {p.returncode}')
            text=(output/'renderer.log').read_text(errors='replace')
            render_lines=[line for line in text.splitlines() if 'renderer' in line.lower()]
            if not render_lines or not all('NVIDIA' in line and '4050' in line and 'Intel' not in line for line in render_lines):
                raise RuntimeError('Actual OpenGL renderer is not verified NVIDIA RTX4050')
        result['status']='PASS'
    except BaseException as exc:
        result.update(status='FAIL',error=repr(exc))
        if watch:watch.halt(exc)
        if p and p.poll() is None:
            p.terminate()
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=3)
    finally:
        result.update(exit_code=p.returncode if p else None,wall_s=time.monotonic()-started)
        if watch:
            result['gpu_summary']={k:dict(min=min(r[k] for r in watch.samples),max=max(r[k] for r in watch.samples))
                for k in ('temperature_c','memory_mib','utilization_pct','power_w')}
            watch.close()
        (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(dict(result=result,output=str(output))),flush=True)
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--stage',required=True);parser.add_argument('--limit',type=float,default=45)
    parser.add_argument('command',nargs=argparse.REMAINDER)
    args=parser.parse_args();cmd=args.command
    if cmd and cmd[0]=='--':cmd=cmd[1:]
    if not cmd:parser.error('command required after --')
    raise SystemExit(run_stage(args.stage,cmd,args.limit))

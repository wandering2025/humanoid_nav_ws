"""Serial, fail-fast live-demo checks with every attempted run retained.

Run through scripts/env.sh with the configured venv interpreter. This evaluator
only launches the normal demo, reads evidence and signals its own launcher.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

WS = Path(__file__).resolve().parents[2]


def save(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data,indent=2)+'\n')
    temporary.replace(path)


def stop_child(child):
    if child and child.poll() is None:
        child.terminate()
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            child.kill(); child.wait(timeout=5)


def run_case(args, label):
    case = args.output/label
    case.mkdir(parents=True,exist_ok=False)
    started = time.monotonic()
    baseline = set((WS/'run_data/exploration').glob('*_exploration_*'))
    cmd = [str(WS/'scripts/run_exploration_demo.sh')]
    if args.hold:
        cmd.append('--hold')
    info = dict(label=label,profile=args.profile,cancel=args.cancel,hold_s=args.hold,
                started_utc=datetime.now(timezone.utc).isoformat(),command=cmd,
                code_sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip(),
                run=None,status='RUNNING')
    save(case/'case.json',info)
    child = load = None
    run = None
    arrived_at = signaled_at = None
    load_stopped = False
    with (case/'terminal.log').open('w') as log, (case/'load.log').open('w') as load_log:
        try:
            child = subprocess.Popen(cmd,cwd=args.cwd,stdout=log,stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL,start_new_session=True)
            info['launcher_pid']=child.pid; save(case/'case.json',info)
            print(f'{label}: launcher {child.pid}',flush=True)
            next_progress=0.
            while child.poll() is None:
                now=time.monotonic()
                if run is None:
                    new=set((WS/'run_data/exploration').glob('*_exploration_*'))-baseline
                    if len(new)>1:
                        raise RuntimeError('Ambiguous concurrent run directories')
                    if new:
                        run=new.pop();info['run']=str(run);save(case/'case.json',info)
                if run:
                    ready=all((run/f'ready-{name}.png').exists() for name in ('mujoco','rviz'))
                    if ready and args.profile=='share-proxy' and load is None:
                        load=subprocess.Popen([str(Path(os.environ['T1_VENV'])/'bin/python'),
                            str(WS/'scripts/exploration/share_load.py'),'--run',str(run),
                            '--output',str(case/'capture')],stdout=load_log,stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL,start_new_session=True)
                        info['load_pid']=load.pid;save(case/'case.json',info)
                    arrived=run/'arrived-rviz.png'
                    if arrived.exists() and arrived.read_bytes().endswith(b'IEND\xaeB`\x82'):
                        arrived_at=arrived_at or now
                    if arrived_at and load and not load_stopped and now-arrived_at>=max(2.,args.hold):
                        stop_child(load);load_stopped=True
                    if load and load.poll() is not None and not load_stopped:
                        raise RuntimeError(f'Capture/encode load exited early: {load.returncode}')
                    if not signaled_at:
                        should_stop=bool(arrived_at and args.hold and now-arrived_at>=args.hold)
                        if args.cancel=='startup':
                            trace=WS/'run_data'/run.name/'sim_trace.csv'
                            should_stop=trace.exists() and trace.stat().st_size>10000 and not ready
                        elif args.cancel=='moving':
                            decisions=run/'explorer/decisions.jsonl'
                            if decisions.exists():
                                rows=[json.loads(line) for line in decisions.read_text().splitlines()]
                                should_stop=any(r.get('event')=='navigate' and r.get('sim',0)>=35 for r in rows)
                        if should_stop:
                            child.send_signal(signal.SIGINT);signaled_at=now
                            info['signal']=dict(kind='SIGINT',elapsed_s=now-started,reason=args.cancel or 'hold_complete')
                            save(case/'case.json',info)
                if now-started>1800 and not signaled_at:
                    child.send_signal(signal.SIGINT);signaled_at=now
                    info['timeout']=True;save(case/'case.json',info)
                if signaled_at and now-signaled_at>180:
                    raise TimeoutError('Owned launcher did not finish bounded cleanup')
                if now>=next_progress:
                    lines=(case/'terminal.log').read_text(errors='replace').splitlines()
                    print(f'{label} {now-started:.0f}s: '+(lines[-1][:240] if lines else 'starting'),flush=True)
                    next_progress=now+25.
                time.sleep(.2)
            info['launcher_exit']=child.returncode
        except BaseException as exc:
            info['harness_error']=repr(exc)
            if child and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=180)
                except subprocess.TimeoutExpired:
                    info['remaining_launcher_pid']=child.pid
        finally:
            stop_child(load)
            info['load_exit']=load.returncode if load else None
            info.update(finished_utc=datetime.now(timezone.utc).isoformat(),wall_s=time.monotonic()-started)
            if child:
                info['launcher_exit']=child.poll()
    final=run/'final-result.json' if run else None
    if final and final.exists():
        result=json.loads(final.read_text())
        info['mission_status']=result['status']
        info['readiness_wall_s']=result.get('readiness_wall_s')
        info['dynamic']=result.get('dynamic')
        info['final_error_m']=result.get('truth_final_hold_max_error_m')
        clean=bool(result.get('dds_port_released') and not result.get('remaining_owned_pids')
                   and not result.get('remaining_project_processes')
                   and result.get('safe_stop',{}).get('physical_stationary')
                   and not result.get('fell') and not any(result.get('contacts',{}).values())
                   and not result.get('gpu_monitor_error') and not result.get('cleanup_error'))
        if args.cancel:
            info['expected_cancellation']=bool(info.get('signal') and result['status']=='FAILED'
                                               and clean and info['launcher_exit']!=0)
            info['status']='PASS' if info['expected_cancellation'] else 'FAIL'
        else:
            # Independent path, dynamic, graphics and timing checks after all
            # live processes have exited; audit failures stop the batch.
            audit=subprocess.run([sys.executable,str(WS/'scripts/exploration/gui_report.py'),
                str(run),str(case/'gui-audit.json')],capture_output=True,text=True)
            (case/'audit.log').write_text(audit.stdout+audit.stderr)
            info['audit_exit']=audit.returncode
            info['status']='PASS' if result['status']=='SUCCESS' and clean and audit.returncode==0 and info['launcher_exit']==0 else 'FAIL'
    else:
        info['status']='FAIL';info['missing_final_result']=True
    if args.profile=='share-proxy':
        capture=case/'capture/result.json'
        if capture.exists():
            data=json.loads(capture.read_text());info['capture']=data
            if data['status']!='STOPPED' or data['samples']<10 or info['load_exit']!=0:
                info['status']='FAIL'
        else:
            info['status']='FAIL'
    if info.get('harness_error') or info.get('timeout'):
        info['status']='FAIL'
    save(case/'case.json',info)
    print(json.dumps({k:info.get(k) for k in ('label','status','mission_status','run','wall_s','harness_error')}) ,flush=True)
    return info['status']=='PASS'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--prefix',required=True)
    parser.add_argument('--profile',choices=('default','share-proxy'),default='default')
    parser.add_argument('--count',type=int,default=1)
    parser.add_argument('--start-index',type=int,default=1)
    parser.add_argument('--hold',type=float,default=0.)
    parser.add_argument('--cancel',choices=('startup','moving'))
    parser.add_argument('--cwd',type=Path,default=WS)
    args=parser.parse_args();args.output=args.output.resolve()
    if args.count<1 or args.hold<0 or (args.cancel and args.hold):
        parser.error('invalid case count/hold/cancellation')
    for index in range(args.start_index,args.start_index+args.count):
        if not run_case(args,f'{args.prefix}-{index:02d}'):
            return 1
    return 0


if __name__=='__main__':
    raise SystemExit(main())

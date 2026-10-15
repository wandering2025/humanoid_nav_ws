"""Independent serial window reads, then offline CPU split-view H264 encoding.

No ROS participants, renderer, window manipulation, GPU encoder or demo control.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time


def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def windows():
    tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True,timeout=3)
    result={}
    for name in ('MuJoCo','RViz'):
        ids=[int(m.group(1),16) for line in tree.splitlines()
             if (m:=re.match(r'\s*(0x[0-9a-f]+) "([^\"]+)"',line)) and name in m.group(2)]
        if len(ids)>1:raise RuntimeError(f'Ambiguous {name} windows')
        result[name]=ids[0] if ids else None
    return result


def capture(run,out):
    out.mkdir(parents=True,exist_ok=False)
    sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    assert not subprocess.check_output(['git','status','--porcelain'],text=True).strip(),'Record clean source only'
    started=time.monotonic();frames=[];error=None;ids=None
    try:
        while time.monotonic()-started<1000:
            if ids is None:
                ready=all((run/f'ready-{name}.png').exists() and
                    (run/f'ready-{name}.png').read_bytes().endswith(b'IEND\xaeB`\x82')
                    for name in ('mujoco','rviz'))
                if not ready:
                    if time.monotonic()-started>120:raise RuntimeError('Ready screenshot deadline')
                    time.sleep(.1);continue
                ids=windows();assert all(ids.values()),'Both validated demo windows required'
            if windows()!=ids:break
            now=time.monotonic();entry=dict(index=len(frames),mono=now,utc=datetime.now(timezone.utc).isoformat(),images={})
            for name,wid in ids.items():
                # Sequential bounded reads of already-running, validated windows.
                p=out/f'{len(frames):05d}-{name.lower()}.png';begin=time.monotonic()
                r=subprocess.run(['import','-window',str(wid),'-depth','8',str(p)],capture_output=True,timeout=3,
                    env=dict(os.environ,MAGICK_THREAD_LIMIT='1',OMP_NUM_THREADS='1'))
                if r.returncode:raise RuntimeError(f'{name} capture exit{r.returncode}: {r.stderr.decode(errors="replace")}')
                entry['images'][name]=dict(file=p.name,mono=begin,finished=time.monotonic(),bytes=p.stat().st_size,sha256=digest(p))
            frames.append(entry)
            with (out/'frames.jsonl').open('a') as f:f.write(json.dumps(entry)+'\n')
            partial=run/'partial-result.json'
            if partial.exists():
                status=json.loads(partial.read_text()).get('status')
                if status=='ARRIVED_SAFE_STOP' and time.time()-partial.stat().st_mtime>5:break
            time.sleep(max(0,.25-(time.monotonic()-now)))
        else:raise RuntimeError('Capture deadline')
    except Exception as exc:error=repr(exc)
    meta=dict(source_sha=sha,run_id=run.name,run_dir=str(run),start_mono=started,end_mono=time.monotonic(),
        frame_pairs=len(frames),window_ids=ids,error=error,
        method='Independent ImageMagick serial reads of MuJoCo and RViz, <=4 paired frames/s. No encoding during simulation.')
    (out/'capture.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta),flush=True)
    return 0 if error is None and len(frames)>50 else 1


def encode(run,out):
    import cv2
    import numpy as np
    r=json.loads((run/'final-result.json').read_text());meta=json.loads((out/'capture.json').read_text())
    assert r['status']=='SUCCESS' and r['arrived'] and r['dynamic']['verdict']=='PASS'
    assert r['contacts']==dict(static_env=0,obstacle=0,ground_nonfoot=0) and not r['fell']
    assert r['safe_stop']['physical_stationary'] and r['dds_port_released'] and not r['remaining_owned_pids']
    assert meta['source_sha']==r['code_sha'] and meta['error'] is None
    frames=[json.loads(l) for l in (out/'frames.jsonl').read_text().splitlines()]
    first=[cv2.imread(str(out/frames[0]['images'][name]['file'])) for name in ('MuJoCo','RViz')]
    widths=[v.shape[1] for v in first];h=max(v.shape[0] for v in first);w=sum(widths)
    h+=h%2;w+=w%2;fps=4.;path=out/'advanced-demo.mp4'
    writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'avc1'),fps,(w,h+112))
    assert writer.isOpened(),'Existing CPU H264 encoder unavailable'
    begin=frames[0]['mono'];count=int((frames[-1]['mono']-begin)*fps)+1;idx=0;cached=-1
    for n in range(count):
        now=begin+n/fps
        while idx+1<len(frames) and frames[idx+1]['mono']<=now:idx+=1
        if cached!=idx:
            views=[cv2.imread(str(out/frames[idx]['images'][name]['file'])) for name in ('MuJoCo','RViz')]
            cached=idx
        canvas=np.full((h+112,w,3),24,np.uint8);x=0
        for im in views:
            ih,iw=im.shape[:2];canvas[80:80+ih,x:x+iw]=im;x+=iw
        for text,y,size in ((f'Booster T1 | Guided autonomous mapping + SLAM navigation | Live elapsed {n/fps:.1f}s',29,.7),
             ('MuJoCo physical walking                                            RViz: live LiDAR, growing map, planned path',59,.6),
             ('Ideal truth odometry and attitude compensation | '+meta['source_sha'],h+102,.5)):
            cv2.putText(canvas,text,(16,y),cv2.FONT_HERSHEY_SIMPLEX,size,(225,235,240),1,cv2.LINE_AA)
        writer.write(canvas)
    card=np.full((h+112,w,3),24,np.uint8)
    lines=['POST-RUN VERIFIED EVIDENCE (not a live overlay)',
           'Guided autonomous mapping -> return to A -> nonstraight A-to-B navigation: SUCCESS',
           'Dynamic obstacle response: '+r['dynamic']['response'],
           'Static contacts 0 | Dynamic contacts 0 | Non-foot ground contacts 0 | Falls 0',
           'Safety stop + physical standstill: PASS | Registered processes cleaned | DDS port released',
           'SLAM is the sole map->odom owner. Map saved from actual LiDAR SLAM output.',
           'Limits: ideal odometry, truth attitude, flat wide scene, reactive kinematic obstacles.',
           'Not frontier exploration, not hardware validation, not a reliability certification.',
           run.name,meta['source_sha']]
    for i,line in enumerate(lines):cv2.putText(card,line,(24,70+i*48),cv2.FONT_HERSHEY_SIMPLEX,.66,(220,235,240),1,cv2.LINE_AA)
    for _ in range(32):writer.write(card)
    writer.release()
    cap=cv2.VideoCapture(str(path));decoded=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        decoded+=1
    cap.release();assert decoded==count+32
    verification=dict(path=str(path),bytes=path.stat().st_size,sha256=digest(path),source_sha=meta['source_sha'],run_id=run.name,
        width=w,height=h+112,fps=fps,live_seconds=count/fps,evidence_card_seconds=8,total_seconds=decoded/fps,
        decoded_frames=decoded,expected_frames=count+32,capture_pairs=len(frames),
        max_pair_gap_s=max(b['mono']-a['mono'] for a,b in zip(frames,frames[1:])),
        max_view_skew_s=max(abs(f['images']['MuJoCo']['mono']-f['images']['RViz']['mono']) for f in frames),
        full_decode_verified=True,visual_review_pending=True)
    (out/'video-verification.json').write_text(json.dumps(verification,indent=2)+'\n')
    print(json.dumps(verification),flush=True);return 0


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('capture','encode'))
    p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();raise SystemExit((capture if a.mode=='capture' else encode)(a.run_dir,a.output))

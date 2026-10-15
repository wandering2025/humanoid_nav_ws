"""Independent existing ImageMagick capture; offline CPU encoding after demo exit.

Does not launch/control ROS, MuJoCo, windows, renderers or GPU APIs.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time


def windows():
    tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True,timeout=3)
    return [int(m.group(1),16) for line in tree.splitlines()
            if (m:=re.match(r'\s*(0x[0-9a-f]+) "MuJoCo[^\"]*"',line))]


def capture(run,output):
    output.mkdir(parents=True,exist_ok=False)
    start=time.monotonic();frames=[];wid=None;error=None
    source=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
    if subprocess.check_output(['git','status','--porcelain'],text=True).strip():
        raise RuntimeError('Recording requires a committed source tree')
    # One window at its existing size; no resizing, GL readbacks or encoder load.
    while time.monotonic()-start<210:
        partial=run/'partial-result.json'
        if partial.exists() and time.time()-partial.stat().st_mtime>=7:break
        if wid is None:
            # X11 announces a window before its drawable is readable. The
            # unchanged demo captures MuJoCo then RViz before sending the goal.
            # Wait for both complete PNGs instead of probing the new window.
            ready = all((run/name).is_file() and
                        (run/name).read_bytes().endswith(b'IEND\xaeB`\x82')
                        for name in ('ready-mujoco.png','ready-rviz.png'))
            if not ready:
                if time.monotonic()-start>60:raise RuntimeError('Demo ready screenshots not complete')
                time.sleep(.05);continue
        ids=windows()
        if wid is None and ids:
            if len(ids)!=1:raise RuntimeError('Ambiguous MuJoCo window ownership')
            wid=ids[0]
        if wid is None:
            if time.monotonic()-start>45:raise RuntimeError('MuJoCo window not found')
            time.sleep(.2);continue
        if wid not in ids:break
        stamp=time.monotonic();name=f'{len(frames):05d}.png';p=output/name
        result=subprocess.run(['import','-window',str(wid),'-depth','8',str(p)],capture_output=True,timeout=3,
                              env=dict(os.environ,MAGICK_THREAD_LIMIT='1',OMP_NUM_THREADS='1'))
        if result.returncode:
            error=dict(returncode=result.returncode,stderr=result.stderr.decode(errors='replace'))
            break
        entry=dict(file=name,mono=stamp,finished_mono=time.monotonic(),utc=datetime.now(timezone.utc).isoformat(),
                   bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
        frames.append(entry)
        with (output/'frames.jsonl').open('a') as f:f.write(json.dumps(entry)+'\n')
        time.sleep(max(0,.125-(time.monotonic()-stamp)))
    if time.monotonic()-start>=210:error=dict(reason='capture deadline')
    meta=dict(source_sha=source,run_dir=str(run),window_id=wid,frames=len(frames),start_mono=start,
              end_mono=time.monotonic(),capture_error=error,method='Existing ImageMagick import, one MuJoCo window, up to 8fps. No encoder or second capture stream during simulation.',release_verified=False)
    (output/'capture.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta),flush=True)
    return 0 if len(frames)>50 and error is None else 1


def encode(run,output):
    import cv2
    import numpy as np
    result=json.loads((run/'final-result.json').read_text())
    if result['status'] not in ('SUCCESS','SUCCESS_WITH_WARNING'):
        raise RuntimeError('Refuse to label a failed run as a release recording')
    assert result['arrived'] and result['dynamic']['verdict']=='PASS'
    assert not any(result['contacts'].values()) and result['fell'] is False
    assert result['safe_stop']['ack'] and result['safe_stop']['physical_stationary']
    assert not result['remaining_owned_pids'] and not result['remaining_project_processes'] and result['dds_port_released']
    frames=[json.loads(x) for x in (output/'frames.jsonl').read_text().splitlines()]
    meta=json.loads((output/'capture.json').read_text())
    assert meta['source_sha']==result['code_sha'] and meta['capture_error'] is None
    first=cv2.imread(str(output/frames[0]['file']));h,w=first.shape[:2]
    fps=8.;path=output/'demo.mp4';writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'avc1'),fps,(w,h+96))
    if not writer.isOpened():raise RuntimeError('Existing CPU H264 codec unavailable')
    begin=frames[0]['mono'];index=0;last=first
    # Preserve real wall time: duplicate the latest available frame, no speed-up.
    count=int((frames[-1]['mono']-begin)*fps)+1
    for n in range(count):
        now=begin+n/fps
        while index+1<len(frames) and frames[index+1]['mono']<=now:
            index+=1;last=cv2.imread(str(output/frames[index]['file']))
        canvas=np.full((h+96,w,3),24,np.uint8);canvas[72:72+h]=last
        cv2.putText(canvas,'Booster T1 | MuJoCo | LiDAR + ROS 2 Nav2',(16,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(235,235,235),1,cv2.LINE_AA)
        cv2.putText(canvas,f'Live capture | A START -> B GOAL | elapsed {n/fps:.1f}s',(16,54),cv2.FONT_HERSHEY_SIMPLEX,.50,(200,215,225),1,cv2.LINE_AA)
        cv2.putText(canvas,meta['source_sha'],(16,h+89),cv2.FONT_HERSHEY_SIMPLEX,.4,(180,180,180),1,cv2.LINE_AA)
        writer.write(canvas)
    # Explicit post-run evidence card, separate from captured live imagery.
    card=np.full((h+96,w,3),24,np.uint8)
    lines=['POST-RUN VERIFIED RESULT',result['status'],'Navigation: SUCCESS | Dynamic obstacle check: PASS',
           'Contacts: 0 | Falls: 0 | Safe stop: PASS','Owned processes reaped | DDS port released',
           'U01: safe wait for approaching obstacle, then resume',
           'Ideal truth odometry; reactive navigation; flat ground',
           'W13 historical acceptance: FAILED (36/40)',run.name,meta['source_sha']]
    if result['status']=='SUCCESS_WITH_WARNING':lines.insert(5,f'DDS teardown warning: exact exit {result["dds_returncode"]}')
    for i,line in enumerate(lines):cv2.putText(card,line,(20,70+i*43),cv2.FONT_HERSHEY_SIMPLEX,.52,(220,230,235),1,cv2.LINE_AA)
    for _ in range(64):writer.write(card)
    writer.release()
    cap=cv2.VideoCapture(str(path));decoded=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        decoded+=1
    cap.release()
    expected=count+64
    if decoded!=expected:raise RuntimeError(f'Decode mismatch {decoded}/{expected}')
    verification=dict(path=str(path),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      fps=fps,width=w,height=h+96,decoded_frames=decoded,expected_frames=expected,
                      live_seconds=count/fps,post_run_evidence_card_seconds=8,source_sha=meta['source_sha'],run_id=run.name,
                      capture_frames=len(frames),max_capture_gap_s=max(b['mono']-a['mono'] for a,b in zip(frames,frames[1:])),
                      capture_error=meta['capture_error'],full_decode_verified=True,visual_review_pending=True)
    (output/'video-verification.json').write_text(json.dumps(verification,indent=2)+'\n');print(json.dumps(verification),flush=True)
    return 0


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('capture','encode'));p.add_argument('--run-dir',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    raise SystemExit((capture if a.mode=='capture' else encode)(a.run_dir,a.output))

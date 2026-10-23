"""Local window-capture/H264 load proxy; not a meeting or network test.

Only reads the two existing simulation windows after their ready screenshots.
Uses existing ImageMagick and OpenCV software encoding. No ROS, new renderer,
window manipulation, GPU encoder, external connection or package installation.
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
import resource

WS = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(WS/'scripts/demo'), str(WS/'scripts/advanced')]
STOP = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=1000.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    def stop(*_):
        global STOP
        STOP = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    from cpu_guard import enter
    guard = enter(software_display=True)
    import cv2
    import numpy as np
    from record import windows
    cv2.setNumThreads(1)
    result = dict(started_utc=datetime.now(timezone.utc).isoformat(),
                  run=str(args.run), method='serial ImageMagick window reads + CPU H264',
                  proxy_only=True, meeting_application_tested=False, guard=guard,
                  requested_fps=15, width=1920, height=1080, samples=0, encoded_frames=0)
    writer = None
    begun = time.monotonic()
    frames = args.output/'samples.jsonl'
    try:
        while not STOP:
            ready = all((args.run/f'ready-{name}.png').exists() and
                (args.run/f'ready-{name}.png').read_bytes().endswith(b'IEND\xaeB`\x82')
                for name in ('mujoco', 'rviz'))
            if ready:
                break
            if time.monotonic()-begun > 120:
                raise TimeoutError('ready screenshots unavailable')
            time.sleep(.1)
        if STOP:
            raise RuntimeError('stopped before capture started')
        ids = windows()
        if not all(ids.values()):
            raise RuntimeError(f'Both existing windows required: {ids}')
        result['window_ids'] = ids
        writer = cv2.VideoWriter(str(args.output/'share-proxy.mp4'),
            cv2.VideoWriter_fourcc(*'avc1'), 15., (1920,1080))
        if not writer.isOpened():
            raise RuntimeError('Existing CPU H264 encoder unavailable')
        start = time.monotonic()
        result['capture_started_mono'] = start
        while not STOP and time.monotonic()-start < args.seconds:
            tick = time.monotonic()
            canvas = np.zeros((1080,1920,3), dtype=np.uint8)
            for index, name in enumerate(('MuJoCo','RViz')):
                raw = subprocess.run(['import','-window',str(ids[name]),'-depth','8','ppm:-'],
                    capture_output=True, timeout=3, check=True,
                    env=dict(os.environ, MAGICK_THREAD_LIMIT='1'))
                frame = cv2.imdecode(np.frombuffer(raw.stdout,dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError('Window capture decode failed')
                height,width = frame.shape[:2]
                scale = min(960/width,1080/height)
                frame = cv2.resize(frame,(int(width*scale),int(height*scale)))
                y=(1080-frame.shape[0])//2; x=index*960+(960-frame.shape[1])//2
                canvas[y:y+frame.shape[0],x:x+frame.shape[1]]=frame
            # Preserve wall duration when serial capture is slower than 15 fps.
            # Report unique samples separately; repeats are never called new captures.
            due = int((time.monotonic()-start)*15)+1
            while result['encoded_frames'] < due:
                writer.write(canvas); result['encoded_frames'] += 1
            result['samples'] += 1
            if result['samples'] == 1:
                cv2.imwrite(str(args.output/'first.png'),canvas)
            row=dict(mono=tick, duration_s=time.monotonic()-tick,
                     samples=result['samples'],encoded_frames=result['encoded_frames'])
            with frames.open('a') as stream:
                stream.write(json.dumps(row)+'\n')
            time.sleep(max(0,1/15-(time.monotonic()-tick)))
        result.update(status='STOPPED' if STOP else 'DEADLINE',
                      capture_wall_s=time.monotonic()-start)
    except Exception as exc:
        result.update(status='FAILED',error=repr(exc))
    finally:
        if writer is not None:
            writer.release()
        result['finished_utc']=datetime.now(timezone.utc).isoformat()
        result['wall_s']=time.monotonic()-begun
        if result.get('capture_wall_s'):
            result['unique_capture_fps']=result['samples']/result['capture_wall_s']
            result['encoded_fps']=result['encoded_frames']/result['capture_wall_s']
        result['self_cpu_s']=time.process_time()
        children=resource.getrusage(resource.RUSAGE_CHILDREN)
        result['capture_children_cpu_s']=children.ru_utime+children.ru_stime
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result['status']=='STOPPED' and result['samples']>10 else 1


if __name__ == '__main__':
    raise SystemExit(main())

"""Real-time, uncut dual-window capture. Existing CPU OpenCV/FFmpeg H264 only.

Runs the literal ./scripts/run_demo.sh command in a fresh environment. The lower
panel displays the actual command stdout/stderr. Never re-renders physics data.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

import cv2
import numpy as np
from windows import XWindows, find_windows

WS = Path(__file__).resolve().parents[2]
FPS = 15


def put(canvas, text, xy, scale=.65, color=(225, 232, 239)):
    cv2.putText(canvas, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def capture(output, command):
    output.mkdir(parents=True, exist_ok=False)
    video = output/'demo.mp4'
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'avc1'), FPS, (1920, 1080))
    if not writer.isOpened(): raise RuntimeError('Existing CPU H264 encoder unavailable')
    x11 = XWindows()
    updates = queue.Queue(); lines = []; child = None; interrupted = False
    code_sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=WS, text=True).strip()
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=WS, text=True).strip()
    if dirty: raise RuntimeError('Recording requires a committed clean release candidate')
    allowed = ('HOME','USER','LOGNAME','SHELL','PATH','DISPLAY','WAYLAND_DISPLAY','XDG_RUNTIME_DIR',
               'PULSE_SERVER','WSL_INTEROP','WSL_DISTRO_NAME','PYTHONPATH','LANG','TERM')
    env = {key: os.environ[key] for key in allowed if key in os.environ}
    env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/lib/wsl/lib'
    cv2.setNumThreads(1)
    started = time.monotonic(); index = 0; repeats = 0; last_frame = None; finished = None
    captures = {'MuJoCo': 0, 'RViz': 0}; windows = {}; next_discovery = 0; max_capture_s = 0.
    child = subprocess.Popen(command, cwd=WS, env=env, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
    def read():
        with (output/'console.log').open('w') as log:
            for line in child.stdout:
                print(line, end='', flush=True); log.write(line); log.flush(); updates.put(line.rstrip())
    reader = threading.Thread(target=read, daemon=True); reader.start()
    def stop(*_):
        nonlocal interrupted
        interrupted = True
        if child.poll() is None: child.send_signal(signal.SIGTERM)
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try:
        with (output/'frames.jsonl').open('w') as timing:
            while True:
                now = time.monotonic(); elapsed = now-started
                while not updates.empty(): lines.append(updates.get_nowait())
                if child.poll() is not None and finished is None: finished = now
                if finished and now-finished > 4: break
                if elapsed > 600:
                    stop(); raise TimeoutError('Recording exceeded 600s; demo stop requested')
                if now >= next_discovery:
                    windows = find_windows(); next_discovery = now+1
                canvas = np.full((1080, 1920, 3), (26, 22, 18), np.uint8)
                put(canvas, 'BOOSTER T1  /  HUMANOID LIDAR NAVIGATION', (28, 44), 1.0)
                put(canvas, 'MuJoCo physics + pretrained walking policy', (28, 83), .66, (222, 191, 110))
                put(canvas, 'ROS 2 Jazzy / AMCL + Nav2 / live LiDAR', (1150, 83), .60, (180, 225, 180))
                frame_capture = {}
                for name, (left, width) in {'MuJoCo': (20, 1100), 'RViz': (1140, 760)}.items():
                    image = x11.capture(windows[name]) if name in windows else None
                    if image is None:
                        put(canvas, f'{name}: waiting for window / closed during cleanup', (left+20, 470), .6)
                    else:
                        h, w = image.shape[:2]; ratio = min(width/w, 740/h)
                        image = cv2.resize(image, (round(w*ratio), round(h*ratio)), interpolation=cv2.INTER_AREA)
                        top = 100+(740-image.shape[0])//2; x = left+(width-image.shape[1])//2
                        canvas[top:top+image.shape[0], x:x+image.shape[1]] = image
                        captures[name] += 1; frame_capture[name] = dict(window=windows[name], width=w, height=h)
                put(canvas, f'LIVE WINDOW CAPTURE  |  wall {elapsed:06.1f}s  |  1x speed  |  source {code_sha[:12]}', (28, 878), .63)
                put(canvas, '$ '+' '.join(command), (28, 910), .62, (175, 225, 175))
                for j, line in enumerate(lines[-5:]): put(canvas, line[:215], (28, 940+j*27), .47)
                # Preserve real wall duration if capture/CPU encoding briefly falls behind.
                due = int((time.monotonic()-started)*FPS)
                while index < due and last_frame is not None:
                    writer.write(last_frame); index += 1; repeats += 1
                writer.write(canvas); index += 1; last_frame = canvas
                duration = time.monotonic()-now; max_capture_s = max(max_capture_s, duration)
                timing.write(json.dumps(dict(frame=index-1,wall_s=elapsed,capture_s=duration,windows=frame_capture))+'\n')
                time.sleep(max(0, started+index/FPS-time.monotonic()))
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGTERM)
            try: child.wait(timeout=90)
            except subprocess.TimeoutExpired: pass  # no broad kill; preserve exact owned PID
        reader.join(timeout=5); writer.release(); x11.close()
        manifest = dict(command=command, source_sha=code_sha, source_dirty=dirty, child_pid=child.pid,
            child_exit=child.poll(), interrupted=interrupted, created_utc=datetime.now(timezone.utc).isoformat(),
            video=str(video), bytes=video.stat().st_size, sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
            fps=FPS, size=[1920,1080], frames=index, duration_s=index/FPS, wall_s=time.monotonic()-started,
            repeated_frames=repeats, captures=captures, max_capture_and_encode_s=max_capture_s,
            method='Live X11 window pixels, simultaneous two-panel composition, CPU H264, uncut wall-time timeline')
        (output/'capture.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(json.dumps(manifest,indent=2),flush=True)
    return child.returncode or 0


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--startup-check',action='store_true');a=p.parse_args()
    raise SystemExit(capture(a.output,['./scripts/run_demo.sh']+(['--startup-check'] if a.startup_check else [])))

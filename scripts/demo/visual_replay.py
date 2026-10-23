"""Bounded CPU-only saved-map display check. No simulator or navigation nodes."""
from cpu_guard import enter
GUARD=enter(software_display=True)

import argparse
import hashlib
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import numpy as np
import yaml
import rclpy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile,DurabilityPolicy
from grid_visuals import cloud
from gpu_watch import ERROR

WS=Path(__file__).resolve().parents[2]


def read_pgm(path):
    with path.open('rb') as f:
        if f.readline().strip()!=b'P5':raise ValueError('Expected binary PGM')
        header=[]
        while len(header)<3:
            header.extend(f.readline().split(b'#',1)[0].split())
        width,height,maximum=map(int,header)
        if maximum!=255:raise ValueError('Expected 8-bit PGM')
        return np.frombuffer(f.read(),dtype=np.uint8).reshape(height,width)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    processes=[];node=None;result=dict(guard=GUARD,scope='CPU saved-map display only; no navigation acceptance')
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        with socket.socket() as s:s.bind(('127.0.0.1',42128))
        check=subprocess.run([sys.executable,str(WS/'scripts/demo/cpu_view.py'),'--check'],capture_output=True,text=True,timeout=15)
        (out/'renderer.json').write_text(check.stdout)
        if check.returncode:raise RuntimeError(check.stderr)
        config=yaml.safe_load((WS/'src/t1_navigation/config/demo.rviz').read_text())
        config['Visualization Manager']['Displays']=[d for d in config['Visualization Manager']['Displays'] if d['Class']=='rviz_default_plugins/PointCloud2']
        config['Window Geometry']=dict(Width=900,Height=650)
        (out/'cpu-cells.rviz').write_text(yaml.safe_dump(config,sort_keys=False))
        for name,cmd in [('dds',[str(WS/'scripts/start_local_dds.sh')]),('rviz',[sys.executable,str(WS/'scripts/demo/cpu_view.py'),'-d',str(out/'cpu-cells.rviz')])]:
            with (out/f'{name}.log').open('w') as log:
                p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append((name,p))
            if name=='dds':time.sleep(.5)
        rclpy.init();node=rclpy.create_node('w14_cpu_saved_map_display')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        path=WS/'src/t1_navigation/maps/dynamic_crossing_world.yaml';meta=yaml.safe_load(path.read_text())
        image_path=path.parent/meta['image'];pixels=read_pgm(image_path)[::-1]
        prob=1.-pixels.astype(float)/255.
        values=np.where(prob>meta['occupied_thresh'],100,np.where(prob<meta['free_thresh'],0,-1))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=pixels.shape[1];grid.info.height=pixels.shape[0]
        grid.info.resolution=meta['resolution'];grid.info.origin.position.x=meta['origin'][0];grid.info.origin.position.y=meta['origin'][1];grid.info.origin.orientation.w=1.
        grid.data=values.astype(np.int8).ravel().tolist()
        # All three renderers use the same saved source grid here, not live costmaps.
        pubs=[(node.create_publisher(PointCloud2,'/t1/viz/'+k+'_cells',qos),cloud(grid,k)) for k in ('map','global','local')]
        deadline=time.monotonic()+20;last=0
        while time.monotonic()<deadline:
            if any(p.poll() is not None for _,p in processes):raise RuntimeError('CPU display child exited early')
            text=(out/'rviz.log').read_text(errors='replace')
            if ERROR.search(text) or '[ERROR]' in text:raise RuntimeError('CPU display graphics/error log; stop without retry')
            if time.monotonic()-last>1:
                for pub,msg in pubs:pub.publish(msg)
                last=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.1)
        tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True,timeout=5)
        windows=[line.split()[0] for line in tree.splitlines() if '"' in line and 'RViz' in line]
        if not windows:raise RuntimeError('CPU RViz window missing')
        subprocess.run(['import','-window',windows[-1],str(out/'cpu-cells.png')],check=True,timeout=10)
        result.update(status='PASS',source=str(image_path.relative_to(WS)),source_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(),points={k:msg.width for k,(_,msg) in zip(('map','global','local'),pubs)})
    except BaseException as exc:result.update(status='FAIL',error=repr(exc))
    finally:
        if node:node.destroy_node();rclpy.shutdown()
        result['exit_codes']={}
        for name,p in reversed(processes):
            if p.poll() is None:p.send_signal(signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=4)
            result['exit_codes'][name]=p.returncode
        result['display_verified']=result.get('status')=='PASS'
        if any(code!=0 for code in result['exit_codes'].values()):
            result.update(status='FAIL',cleanup_error='A CPU replay child exited abnormally; display verification alone is not full acceptance')
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    return 0 if result['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())

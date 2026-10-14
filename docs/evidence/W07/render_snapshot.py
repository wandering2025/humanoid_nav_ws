"""Render a recorded ROS truth/joint snapshot in a separate visualization MjData.

No physics is advanced; this is a visual comparison of observed physical motion,
not a motion mechanism. The live simulator is never accessed or modified.
"""
import json, os, subprocess
from pathlib import Path
import mujoco
import numpy as np
def save_png(array,path):
    height,width,_=array.shape
    ppm=f'P6\n{width} {height}\n255\n'.encode()+array.tobytes()
    subprocess.run(['convert','ppm:-',str(path)],input=ppm,check=True)
WS=Path(os.environ['T1_WS']);OUT=WS/'docs/evidence/W07'
result=json.loads((OUT/'ros-rviz.json').read_text());snapshot=json.loads((OUT/'raw/visual-snapshot.json').read_text())
path=result['run']['manifest']['scene_file'];model=mujoco.MjModel.from_xml_path(path);data=mujoco.MjData(model)
row=snapshot['truth'];data.qpos[:7]=row[1:8]
for name,value in snapshot['joints'].items():data.qpos[model.jnt_qposadr[model.joint(name).id]]=value
mujoco.mj_forward(model,data);model.vis.global_.offwidth=1000;model.vis.global_.offheight=800
with mujoco.Renderer(model,height=800,width=1000) as renderer:
    camera=mujoco.MjvCamera();camera.lookat[:]=[3,0,0];camera.distance=14;camera.azimuth=90;camera.elevation=-90
    renderer.update_scene(data,camera);save_png(renderer.render(),OUT/'mujoco-static-top.png')
    camera.lookat[:]=[data.qpos[0],data.qpos[1],.6];camera.distance=3.5;camera.azimuth=40;camera.elevation=-20
    renderer.update_scene(data,camera);save_png(renderer.render(),OUT/'mujoco-robot.png')
print('Rendered actual recorded physical snapshot at sim stamp',snapshot['stamp_ns'])

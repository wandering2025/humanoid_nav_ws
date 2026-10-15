"""A single bounded NVIDIA window; optional static T1 render, no simulation steps."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import time

parser=argparse.ArgumentParser();parser.add_argument('--model');parser.add_argument('--seconds',type=float,default=15)
parser.add_argument('--width',type=int,default=320);parser.add_argument('--height',type=int,default=240)
parser.add_argument('--fps',type=float,default=10);args=parser.parse_args()
import glfw
if not glfw.init():raise RuntimeError('GLFW initialization failed')
window=None;context=None
try:
    glfw.window_hint(glfw.RESIZABLE,False)
    window=glfw.create_window(args.width,args.height,'W14 NVIDIA staged check - close to stop',None,None)
    if not window:raise RuntimeError('GLFW window missing')
    glfw.make_context_current(window);glfw.swap_interval(1)
    get_string=ctypes.CFUNCTYPE(ctypes.c_char_p,ctypes.c_uint)(glfw.get_proc_address('glGetString'))
    renderer=get_string(0x1F01).decode()
    print(json.dumps(dict(renderer=renderer,mode='static-render-only' if args.model else 'clear-only')),flush=True)
    if 'NVIDIA' not in renderer or '4050' not in renderer or 'Intel' in renderer:
        raise RuntimeError('Expected NVIDIA RTX4050; stopping before draw')
    if args.model:
        import mujoco
        model=mujoco.MjModel.from_xml_path(args.model);data=mujoco.MjData(model)
        if model.nkey:mujoco.mj_resetDataKeyframe(model,data,0)
        mujoco.mj_forward(model,data)
        scene=mujoco.MjvScene(model,maxgeom=3000);camera=mujoco.MjvCamera();option=mujoco.MjvOption()
        camera.lookat[:]=model.stat.center;camera.distance=3.;camera.azimuth=135.;camera.elevation=-20.
        context=mujoco.MjrContext(model,mujoco.mjtFontScale.mjFONTSCALE_100)
    else:
        clear_color=ctypes.CFUNCTYPE(None,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float)(glfw.get_proc_address('glClearColor'))
        clear=ctypes.CFUNCTYPE(None,ctypes.c_uint)(glfw.get_proc_address('glClear'))
    start=time.monotonic();frames=0
    while time.monotonic()-start<args.seconds:
        if glfw.window_should_close(window):raise RuntimeError('Canary window closed early')
        glfw.poll_events()
        if args.model:
            mujoco.mjv_updateScene(model,data,option,None,camera,mujoco.mjtCatBit.mjCAT_ALL,scene)
            mujoco.mjr_render(mujoco.MjrRect(0,0,*glfw.get_framebuffer_size(window)),scene,context)
        else:
            clear_color(.08,.16,.22,1.);clear(0x4000)
        glfw.swap_buffers(window);frames+=1
        Path(os.environ['T1_GPU_HEARTBEAT']).write_text(str(frames))
        time.sleep(max(0,start+frames/args.fps-time.monotonic()))
    print(json.dumps(dict(frames=frames,wall_s=time.monotonic()-start)),flush=True)
finally:
    if context:context.free()
    if window:glfw.destroy_window(window)
    glfw.terminate()

"""Four render-only A/B geoms, using the proven production viewer lifecycle."""
import mujoco
import numpy as np

def annotate(viewer):
    scene = viewer.user_scn
    scene.ngeom = 0
    for x, y, label, color in ((0., 0., 'A  START', [.10,.75,.95,1.]),
                               (14., 0., 'B  GOAL', [.20,.95,.45,1.])):
        for kind, size, pos, text in (
            (mujoco.mjtGeom.mjGEOM_CYLINDER,[.65,.008,0.],[x,y,.012],''),
            (mujoco.mjtGeom.mjGEOM_LABEL,[.1,.1,.1],[x,y-.85,.12],label)):
            geom=scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom,kind,size,pos,np.eye(3).ravel(),color)
            geom.label=text
            scene.ngeom+=1
    viewer.cam.distance=8.
    viewer.cam.elevation=-40.
    viewer.cam.azimuth=90.

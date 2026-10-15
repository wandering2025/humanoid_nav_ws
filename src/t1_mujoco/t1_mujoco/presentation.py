"""Small, static annotations on the viewer copy, never the physics model."""
import mujoco
import numpy as np


def style_model(model):
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
        if name.startswith('env_wall_'):
            model.geom_rgba[i] = [.50, .60, .68, 1.]
        elif name.startswith('obstacle_'):
            model.geom_rgba[i] = [1., .32, .06, 1.]
    # Existing headlight only; no new lights, shadow maps or render settings.
    model.vis.headlight.ambient[:] = [.35, .35, .35]
    model.vis.headlight.diffuse[:] = [.65, .65, .65]


def annotate(viewer):
    """Four display geoms, outside MjModel and every collision/ray query."""
    scene = viewer.user_scn
    scene.ngeom = 0
    for x, label, color in ((0., 'A  START', [.10, .75, .95, 1.]),
                            (9., 'B  GOAL', [.20, .95, .45, 1.])):
        for kind, size, pos, text in (
            (mujoco.mjtGeom.mjGEOM_CYLINDER, [.65, .008, 0.], [x, 0., .012], ''),
            (mujoco.mjtGeom.mjGEOM_LABEL, [.1, .1, .1], [x, -.85, .12], label),
        ):
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, kind, size, pos, np.eye(3).ravel(), color)
            geom.label = text
            scene.ngeom += 1
    # Keep W14's robot-following camera; modestly widen to show nearby obstacles.
    viewer.cam.distance = 6.
    viewer.cam.elevation = -35.

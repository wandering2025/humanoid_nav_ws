"""Presentation on the viewer COPY only; no ROS, physics, sensor or map access."""
import math

import mujoco
import numpy as np


def style_model(view_model):
    """Call before creating the viewer context. Never pass the runner model."""
    for i in range(view_model.ngeom):
        name = mujoco.mj_id2name(view_model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
        if view_model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
            view_model.geom_matid[i] = -1
            view_model.geom_rgba[i] = [.22, .28, .34, 1.]
        elif name.startswith('env_wall_'):
            view_model.geom_rgba[i] = [.52, .62, .70, 1.]
        elif name.startswith('env_'):
            view_model.geom_rgba[i] = [.36, .46, .55, 1.]
        elif name.startswith('obstacle_'):
            view_model.geom_rgba[i] = [1., .30, .06, 1.]
    view_model.vis.headlight.ambient[:] = [.35, .35, .35]
    view_model.vis.headlight.diffuse[:] = [.65, .65, .65]
    view_model.vis.headlight.specular[:] = [.12, .12, .12]
    view_model.vis.quality.shadowsize = 1024
    view_model.vis.quality.offsamples = 0


def decorate(viewer):
    """Render-only geoms live outside MjModel, so cannot collide or return rays."""
    scene = viewer.user_scn
    scene.ngeom = 0

    def geom(kind, size, pos, color, label=''):
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, kind, size, pos, np.eye(3).ravel(), color)
        g.label = label
        scene.ngeom += 1
        return g

    def line(a, b, width, color):
        g = geom(mujoco.mjtGeom.mjGEOM_CAPSULE, [width, 0, 0], [0, 0, 0], color)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)

    # A restrained floor grid gives scale without adding physical geometry.
    for x in range(-2, 12):
        line([x, -5.4, .005], [x, 5.4, .005], .006, [.32, .39, .45, 1])
    for y in range(-5, 6):
        line([-2.4, y, .005], [11.4, y, .005], .006, [.32, .39, .45, 1])
    for x, label, color in ((0., 'A  START', [.1, .75, .95, 1.]),
                             (9., 'B  GOAL', [.25, .95, .55, 1.])):
        for j in range(40):
            a, b = 2*math.pi*j/40, 2*math.pi*(j+1)/40
            line([x+.65*math.cos(a), .65*math.sin(a), .016],
                 [x+.65*math.cos(b), .65*math.sin(b), .016], .035, color)
        geom(mujoco.mjtGeom.mjGEOM_LABEL, [.1, .1, .1], [x, -1.05, .12], color, label)
    viewer.cam.lookat[:] = [4.5, 0., .35]
    viewer.cam.distance = 14.8
    viewer.cam.azimuth = 90.
    viewer.cam.elevation = -48.


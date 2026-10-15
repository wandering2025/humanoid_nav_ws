"""Prove presentation cannot alter physics, collision geometry or ray returns."""
import hashlib
from pathlib import Path
import subprocess
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np
from t1_mujoco.presentation import style_model, annotate

WS=Path(__file__).resolve().parents[2]


class PresentationTests(unittest.TestCase):
    def test_view_copy_keeps_dynamics_and_rays_identical(self):
        xml='''<mujoco><worldbody><geom name="floor" type="plane" size="10 10 .1"/>
        <geom name="env_wall_north" type="box" pos="0 3 1" size="3 .1 1"/>
        <body pos="0 0 1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body>
        <body name="obstacle" mocap="true" pos="1 0 .5"><geom name="obstacle_box" type="box" size=".2 .2 .5"/></body>
        </worldbody></mujoco>'''
        physical=mujoco.MjModel.from_xml_string(xml);view=mujoco.MjModel.from_xml_string(xml)
        data=mujoco.MjData(physical);copy=mujoco.MjData(view)
        style_model(view)
        self.assertFalse(np.array_equal(physical.geom_rgba,view.geom_rgba))
        for field in ('geom_pos','geom_size','geom_type','geom_contype','geom_conaffinity','geom_friction','geom_group','body_mass','body_inertia'):
            np.testing.assert_array_equal(getattr(physical,field),getattr(view,field))
        for _ in range(100):mujoco.mj_step(physical,data);mujoco.mj_step(view,copy)
        np.testing.assert_array_equal(data.qpos,copy.qpos)
        np.testing.assert_array_equal(data.qvel,copy.qvel)
        self.assertEqual(data.ncon,copy.ncon)
        for model,state in ((physical,data),(view,copy)):mujoco.mj_forward(model,state)
        ids=[];dist=[]
        for model,state in ((physical,data),(view,copy)):
            idx=np.zeros(1,dtype=np.int32)
            dist.append(mujoco.mj_ray(model,state,np.array([0.,0.,.5]),np.array([1.,0.,0.]),None,True,-1,idx))
            ids.append(idx[0])
        self.assertEqual(dist[0],dist[1]);self.assertEqual(ids[0],ids[1])
        holder=SimpleNamespace(user_scn=mujoco.MjvScene(view,maxgeom=300),cam=mujoco.MjvCamera())
        before=(view.ngeom,view.nbody,view.nq,view.nv);annotate(holder)
        self.assertEqual(before,(view.ngeom,view.nbody,view.nq,view.nv))
        self.assertEqual(holder.user_scn.ngeom,4)
        # mjv_initGeom accepts radius/half-height, then expands the render size.
        np.testing.assert_allclose(holder.user_scn.geoms[0].size,[.65,.65,.008])
        np.testing.assert_allclose(holder.user_scn.geoms[2].size,[.65,.65,.008])
        self.assertEqual([g.label for g in holder.user_scn.geoms[:holder.user_scn.ngeom] if g.label],['A  START','B  GOAL'])

    def test_simulation_changes_are_only_view_copy_hooks(self):
        name='src/t1_mujoco/t1_mujoco/simulation.py'
        actual=(WS/name).read_text()
        actual=actual.replace("            presentation = os.environ.get('T1_DEMO_PRESENTATION') == '1'\n            if presentation:\n                from .presentation import style_model, annotate\n                style_model(view_model)\n",'')
        actual=actual.replace('            if presentation:\n                with viewer.lock():\n                    annotate(viewer)\n','')
        original=subprocess.check_output(['git','show','b23a92465b317ca781d3eb210051126c5786908e:'+name],cwd=WS,text=True)
        self.assertEqual(actual,original)


if __name__=='__main__':unittest.main()

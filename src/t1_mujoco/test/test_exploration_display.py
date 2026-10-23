"""Graphical A/B must use the explorer's resolved anchor, including its drift."""
import math
from types import SimpleNamespace

import mujoco
import pytest

from t1_mujoco.advanced_presentation import annotate
from t1_mujoco.exploration_display import start_markers


def test_resolved_rotated_start_is_preserved_in_render_only_geoms():
    anchor = [2., -3., math.pi/2]
    goal = [1.5, 8.]
    markers = start_markers(dict(event='start', anchor_odom=anchor, goal_odom=goal))
    assert markers == ((2., -3., math.pi/2), (1.5, 8.))
    anchor[0] = 999.  # Incoming mutable message cannot change a displayed anchor.
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="1 1 .1"/></worldbody></mujoco>')
    view = SimpleNamespace(user_scn=mujoco.MjvScene(model, maxgeom=8), cam=mujoco.MjvCamera())
    annotate(view, start=markers[0], goal=markers[1])
    assert view.user_scn.ngeom == 4
    assert tuple(view.user_scn.geoms[0].pos[:2]) == (2., -3.)
    assert tuple(view.user_scn.geoms[2].pos[:2]) == (1.5, 8.)
    assert view.user_scn.geoms[1].label == 'A  START'
    assert view.user_scn.geoms[3].label == 'B  GOAL'


@pytest.mark.parametrize('event', [None, {}, dict(event='decision', target=[1,2]),
    dict(event='start',anchor_odom=[0,0],goal_odom=[1,2]),
    dict(event='start',anchor_odom=[0,0,0],goal_odom=[float('nan'),2]),
    dict(event='start',anchor_odom=[0,False,0],goal_odom=[1,2]),
    dict(event='start',anchor_odom=[0,0,0],goal_odom=[1,2,3])])
def test_unrelated_or_invalid_events_cannot_invent_display_coordinates(event):
    assert start_markers(event) is None

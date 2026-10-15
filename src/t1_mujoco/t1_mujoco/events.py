"""Deterministic kinetic obstacles, written only by the physics owner.

Only the event source reads robot position to trigger a scene. No trajectory or
obstacle pose is given to Nav2. Mocap bodies collide but cannot be pushed.
"""
import hashlib
import json
import math
import threading
from pathlib import Path

import yaml

from .scenes import vector


def load_scenario(path, name):
    source = Path(path)
    data = yaml.safe_load(source.read_text())
    if data.get('schema') != 1 or data.get('scene') not in ('dynamic_crossing_world', 'advanced_world'):
        raise ValueError('invalid events schema/scene')
    scenario = data['scenarios'].get(name)
    if scenario is None:
        raise ValueError('unknown dynamic scenario')
    vector(scenario['start'], 2, 'event start')
    if not math.isfinite(scenario['trigger_x']) or scenario['trigger_x'] < 0:
        raise ValueError('invalid event trigger')
    if type(scenario['seed']) is not int or scenario['seed'] < 0:
        raise ValueError('invalid event seed')
    for leg in scenario['legs']:
        vector(leg['to'], 2, 'event waypoint')
        if not math.isfinite(leg['speed']) or not 0 < leg['speed'] <= .8:
            raise ValueError('event speed must be in (0, .8] m/s')
        if not math.isfinite(leg['hold']) or leg['hold'] < 0:
            raise ValueError('invalid event hold')
    if not scenario['legs']:
        raise ValueError('event needs a trajectory')
    return scenario, dict(path=str(source.resolve()), sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                          scenario=name, seed=scenario['seed'], scene=data['scene'])


class SceneEvents:
    def __init__(self, runner, scenario, log=None):
        self.runner, self.scenario, self.log = runner, scenario, log
        body = runner.mj_model.body('obstacle_'+scenario['slot']).id
        self.mocap_id = int(runner.mj_model.body_mocapid[body])
        if self.mocap_id < 0:
            raise ValueError('obstacle must be a mocap body')
        self.size = (2*runner.mj_model.geom('obstacle_'+scenario['slot']).size).tolist()
        self.enable_requested = threading.Event()
        self.episode = None
        self.triggered_at = None
        self.reset()

    def reset(self):
        self.runner.assert_owner()
        self.episode = self.runner.episode
        self.triggered_at = None
        self.enable_requested.clear()
        if not self.scenario.get('require_enable', False):
            self.enable_requested.set()
        self.position = list(self.scenario['start'])
        self.phase = 'staged'
        self.write()

    def write(self):
        self.runner.mj_data.mocap_pos[self.mocap_id] = [*self.position, self.size[2]/2]
        self.runner.mj_data.mocap_quat[self.mocap_id] = [1, 0, 0, 0]

    def update(self):
        """Called before every 2 ms mj_step; paused time never advances a leg."""
        self.runner.assert_owner()
        if self.episode != self.runner.episode:
            self.reset()
        d = self.runner.mj_data
        if (self.enable_requested.is_set() and self.triggered_at is None and self.runner.state == 'READY'
                and d.qpos[0] >= self.scenario['trigger_x']
                and self.runner.incoming[0] > .1):
            self.triggered_at = float(d.time)
            if self.log:
                self.log.event(dict(scene_event='trigger', episode=self.episode,
                                    sim_time=self.triggered_at, robot_xy=d.qpos[:2].tolist()))
        if self.triggered_at is not None:
            remaining = max(0., float(d.time)-self.triggered_at)
            start = self.scenario['start']
            self.phase = 'finished'
            for index, leg in enumerate(self.scenario['legs']):
                distance = math.dist(start, leg['to'])
                duration = distance/leg['speed']
                if remaining < duration:
                    fraction = remaining/duration
                    self.position = [a+(b-a)*fraction for a, b in zip(start, leg['to'])]
                    self.phase = f'move_{index}'
                    break
                remaining -= duration
                self.position = list(leg['to'])
                if remaining < leg['hold']:
                    self.phase = f'hold_{index}'
                    break
                remaining -= leg['hold']
                start = leg['to']
        self.write()

    def snapshot(self):
        return dict(episode=self.episode, sim_time=float(self.runner.mj_data.time),
                    slot=self.scenario['slot'], position=[*self.position, self.size[2]/2],
                    size=self.size, phase=self.phase, triggered_at=self.triggered_at,
                    enabled=self.enable_requested.is_set())

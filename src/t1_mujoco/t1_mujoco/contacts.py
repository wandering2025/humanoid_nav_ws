"""Environment contact events (acceptance §5/§7), sampled every physics substep.

Read-only on the MjData owner thread; evaluation record only, never fed to
navigation. Robot self-contacts and foot-ground contacts are not events.
"""
import csv

FIELDS = ('episode', 't_start', 't_end', 'robot_geom', 'robot_body', 'other_geom', 'other_body',
          'other_class', 'max_penetration_m', 'n_substeps')
FOOT_BODIES = ('left_ankle_roll_link', 'right_ankle_roll_link')


class ContactMonitor:
    def __init__(self, model, path, policy_dt=0.02, root_body='trunk'):
        import numpy as np
        self.np = np
        self.model = model
        root = model.body(root_body).id
        robot_bodies = model.body_rootid == root
        self.robot_geom = robot_bodies[model.geom_bodyid]
        feet = {model.body(name).id for name in FOOT_BODIES}
        self.foot_geom = np.isin(model.geom_bodyid, list(feet))
        try:
            self.ground = model.geom('ground').id
        except KeyError:
            self.ground = -1
        # Separate events need a gap of >= 2 policy steps (acceptance §7).
        self.gap = 2 * policy_dt
        self.open = {}
        self.counts = dict(static_env=0, obstacle=0, ground_nonfoot=0)
        self.file = open(path, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()

    def name(self, kind, index):
        import mujoco
        obj = mujoco.mjtObj.mjOBJ_GEOM if kind == 'geom' else mujoco.mjtObj.mjOBJ_BODY
        return mujoco.mj_id2name(self.model, obj, int(index)) or f'#{index}'

    def sample(self, data, episode):
        n = data.ncon
        if n:
            con = data.contact
            g1, g2, dist = con.geom1[:n], con.geom2[:n], con.dist[:n]
            r1, r2 = self.robot_geom[g1], self.robot_geom[g2]
            for i in self.np.flatnonzero((r1 != r2) & (dist <= 0)):
                robot, other = (g1[i], g2[i]) if r1[i] else (g2[i], g1[i])
                if other == self.ground:
                    if self.foot_geom[robot]:
                        continue
                    kind = 'ground_nonfoot'
                else:
                    body = self.model.geom_bodyid[other]
                    kind = 'obstacle' if self.model.body_mocapid[body] >= 0 else 'static_env'
                self.update(episode, float(data.time), int(robot), int(other), kind, float(-dist[i]))
        self.flush(float(data.time))

    def update(self, episode, t, robot, other, kind, penetration):
        key = (episode, robot, other)
        event = self.open.get(key)
        if event is None:
            self.counts[kind] += 1
            self.open[key] = dict(episode=episode, t_start=t, t_end=t, robot_geom=robot, other_geom=other,
                                  other_class=kind, max_penetration_m=penetration, n_substeps=1)
        else:
            event.update(t_end=t, max_penetration_m=max(event['max_penetration_m'], penetration),
                         n_substeps=event['n_substeps'] + 1)

    def flush(self, now, force=False):
        for key, event in list(self.open.items()):
            if force or now - event['t_end'] >= self.gap:
                row = dict(event)
                row.update(robot_geom=self.name('geom', event['robot_geom']),
                           other_geom=self.name('geom', event['other_geom']),
                           robot_body=self.name('body', self.model.geom_bodyid[event['robot_geom']]),
                           other_body=self.name('body', self.model.geom_bodyid[event['other_geom']]))
                self.writer.writerow(row)
                del self.open[key]
        if force:
            self.file.flush()

    def close(self):
        if not self.file.closed:
            self.flush(float('inf'), force=True)
            self.file.close()

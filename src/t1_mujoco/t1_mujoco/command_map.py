"""W05 velocity contract: physical v_des -> policy u_pol (interfaces.md §6.2/§6.3).

Pure Python; no MuJoCo or ROS. Guard steps 4-6 only clip, zero or keep a
request. Nothing below the walking minimum is ever raised to a gait command.
"""
import math
from pathlib import Path

import yaml

SCHEMA = 1
AXES = ('vx', 'wz_walking_pos', 'wz_walking_neg', 'wz_in_place_pos', 'wz_in_place_neg')


class CommandMapError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise CommandMapError(message)


def _finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


class Table:
    """Monotone piecewise-linear |v_phys| <-> |u_pol| on a verified interval.

    Rows are (v_phys, u_pol), both strictly increasing and positive. v_phys is
    the conservative (largest observed) physical speed for that u_pol, so the
    robot is expected not to exceed the request. Outside [v_lo, v_hi] the
    caller has already zeroed or clipped; no extrapolation.
    """

    def __init__(self, name, rows):
        _require(isinstance(rows, list) and len(rows) >= 2, f'{name}: need at least two rows')
        for row in rows:
            _require(isinstance(row, list) and len(row) == 2 and all(_finite(x) for x in row),
                     f'{name}: rows must be [v_phys, u_pol] finite pairs')
        v, u = [float(r[0]) for r in rows], [float(r[1]) for r in rows]
        _require(v[0] > 0 and u[0] > 0, f'{name}: first row must be positive (zero is the deadband)')
        _require(all(b > a for a, b in zip(v, v[1:])) and all(b > a for a, b in zip(u, u[1:])),
                 f'{name}: v_phys and u_pol must both be strictly increasing')
        _require(u[-1] <= 1.0, f'{name}: u_pol beyond upstream command bound 1.0')
        self.name, self.v, self.u = name, v, u

    @property
    def v_lo(self):
        return self.v[0]

    @property
    def v_hi(self):
        return self.v[-1]

    @staticmethod
    def _interp(x, xs, ys):
        if x <= xs[0]:
            return ys[0]
        for i in range(1, len(xs)):
            if x <= xs[i]:
                t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
                return ys[i - 1] + t * (ys[i] - ys[i - 1])
        return ys[-1]

    def forward(self, speed):
        """|v_phys| within [v_lo, v_hi] -> |u_pol|."""
        return self._interp(speed, self.v, self.u)

    def inverse(self, u):
        """|u_pol| within [u_lo, u_hi] -> expected |v_phys|."""
        return self._interp(u, self.u, self.v)


class CommandMap:
    def __init__(self, data, source='<memory>'):
        _require(isinstance(data, dict), 'command map must be a mapping')
        _require(data.get('schema') == SCHEMA, f'schema must be {SCHEMA}')
        _require(data.get('calibrated') is True, 'calibrated must be true to accept /cmd_vel')
        for key in ('evidence', 'applicability', 'capabilities', 'envelope', 'tables'):
            _require(key in data, f'missing {key}')
        caps, env, tables = data['capabilities'], data['envelope'], data['tables']
        for key in ('forward', 'reverse_allowed', 'lateral_allowed', 'in_place_turn_allowed'):
            _require(isinstance(caps.get(key), bool), f'capabilities.{key} must be bool')
        _require(caps['forward'], 'forward walking is the minimum capability')
        _require(not caps['reverse_allowed'] and not caps['lateral_allowed'],
                 'reverse/lateral tables are not part of schema 1; keep them disabled')
        for key in ('vx_min', 'vx_max', 'wz_walking_max', 'wz_in_place_min', 'wz_in_place_max'):
            _require(_finite(env.get(key)) and env[key] >= 0, f'envelope.{key} must be finite >=0')
        self.tables = {}
        for axis in AXES:
            if axis.startswith('wz_in_place') and not caps['in_place_turn_allowed']:
                continue
            self.tables[axis] = Table(axis, tables.get(axis))
        vx = self.tables['vx']
        _require(vx.v_lo <= env['vx_min'] < env['vx_max'] <= vx.v_hi,
                 'vx envelope must lie inside the verified vx table')
        for side in ('pos', 'neg'):
            t = self.tables[f'wz_walking_{side}']
            _require(env['wz_walking_max'] <= t.v_hi, f'wz_walking_max beyond wz_walking_{side} table')
            if caps['in_place_turn_allowed']:
                t = self.tables[f'wz_in_place_{side}']
                _require(t.v_lo <= env['wz_in_place_min'] < env['wz_in_place_max'] <= t.v_hi,
                         f'in-place envelope must lie inside wz_in_place_{side} table')
        self.walking_wz_lo = {side: self.tables[f'wz_walking_{side}'].v_lo for side in ('pos', 'neg')}
        self.data, self.source, self.caps, self.env = data, str(source), caps, env

    @classmethod
    def load(cls, path):
        path = Path(path)
        _require(path.is_absolute() and path.is_file(), f'command map must be an existing absolute file: {path}')
        with path.open() as f:
            return cls(yaml.safe_load(f), path)

    # Guard steps 4-6 ---------------------------------------------------------
    def guard(self, v):
        """(vx, vy, wz) physical -> (u_pol, flags). Input already finite/fresh."""
        vx, vy, wz = (float(x) for x in v)
        env, flags = self.env, []
        if vy != 0:
            flags.append('vy_disabled')
            vy = 0.0
        if vx < 0:
            flags.append('reverse_disabled')
            vx = 0.0
        if vx > env['vx_max']:
            flags.append('vx_clipped')
            vx = env['vx_max']
        if 0 < vx < env['vx_min']:
            flags.append('vx_deadband')  # zeroed, never raised
            vx = 0.0
        ux = self.tables['vx'].forward(vx) if vx > 0 else 0.0
        side = 'pos' if wz > 0 else 'neg'
        mag = abs(wz)
        if ux > 0:
            limit = env['wz_walking_max']
            if mag > limit:
                flags.append('wz_clipped')
                mag = limit
            table = self.tables[f'wz_walking_{side}']
            # Below the first verified walking point: proportional down to zero
            # (the line through the origin and the first row), never raised.
            if 0 < mag < table.v_lo:
                uw = mag * table.u[0] / table.v[0]
            else:
                uw = table.forward(mag) if mag > 0 else 0.0
        elif mag > 0:
            if not self.caps['in_place_turn_allowed']:
                flags.append('in_place_disabled')
                uw = 0.0
            elif mag < env['wz_in_place_min']:
                flags.append('wz_deadband')
                uw = 0.0
            else:
                if mag > env['wz_in_place_max']:
                    flags.append('wz_clipped')
                    mag = env['wz_in_place_max']
                uw = self.tables[f'wz_in_place_{side}'].forward(mag)
        else:
            uw = 0.0
        return (ux, 0.0, math.copysign(uw, wz) if uw else 0.0), flags

    def expected(self, u):
        """M^-1: expected physical (vx, vy, wz) of a guarded policy command."""
        ux, _uy, uw = (float(x) for x in u)
        vx = self.tables['vx'].inverse(ux) if ux > 0 else 0.0
        side = 'pos' if uw > 0 else 'neg'
        mag = abs(uw)
        if mag == 0:
            wz = 0.0
        elif ux > 0:
            table = self.tables[f'wz_walking_{side}']
            wz = mag * table.v[0] / table.u[0] if mag < table.u[0] else table.inverse(mag)
        else:
            wz = self.tables[f'wz_in_place_{side}'].inverse(mag)
        return (vx, 0.0, math.copysign(wz, uw) if wz else 0.0)

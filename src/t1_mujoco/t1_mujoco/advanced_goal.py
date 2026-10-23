"""Presentation goal ownership: editable YAML -> validated per-run snapshot."""
import hashlib
import json
import math
import os
from pathlib import Path
import yaml

WS = Path(__file__).resolve().parents[3]
X_RANGE = (11., 12.)
Y_RANGE = (-.5, .5)
YAW = -math.pi / 2
APPROACH_OFFSET_M = 2.5
SELECTION_CLEARANCE_M = 2.4
SNAPSHOT_ENV = 'T1_ADVANCED_GOAL_SNAPSHOT'


def default_goal_file():
    return Path(os.environ.get('T1_WS', WS)) / 'presentation/goal.yaml'


class UniqueKeysLoader(yaml.SafeLoader):
    """Reject duplicate keys rather than silently choosing the last coordinate."""


def unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in result:
            raise ValueError(f'Duplicate goal configuration key: {key}')
        result[key] = loader.construct_object(value_node)
    return result


UniqueKeysLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def validate_selection(data):
    if not isinstance(data, dict) or set(data) != {'schema', 'frame', 'goal'}:
        raise ValueError('goal.yaml requires exactly schema, frame and goal')
    if type(data['schema']) is not int or data['schema'] != 1 or data['frame'] != 'map':
        raise ValueError('Use schema: 1 and frame: map')
    point = data['goal']
    if not isinstance(point, dict) or set(point) != {'x', 'y'}:
        raise ValueError('goal requires exactly x and y; heading is fixed at -90 degrees')
    for name, bounds in (('x', X_RANGE), ('y', Y_RANGE)):
        value = point[name]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'goal.{name} must be a finite number in metres')
        if not bounds[0] <= value <= bounds[1]:
            raise ValueError(f'goal.{name} must be in [{bounds[0]}, {bounds[1]}] metres (inclusive)')
    x, y = float(point['x']), float(point['y'])
    # Check the entire vertical final-approach segment, not just B. Scene geometry
    # is used only to reject unsafe operator selections, never to generate a map.
    scene = yaml.safe_load((WS/'src/t1_mujoco/worlds/advanced_world.yaml').read_text())
    event = yaml.safe_load((WS/'src/t1_bringup/config/advanced_events.yaml').read_text())['scenarios']['ADV01']
    slot = next(s for s in scene['obstacle_slots'] if s['name'] == event['slot'])
    boxes = [(b['center'][:2], b['size'][:2]) for b in scene['boxes']]
    boxes.append((event['start'], slot['size'][:2]))
    xmin, xmax, ymin, ymax = scene['bounds'];end_y = y + APPROACH_OFFSET_M
    distances = [x-xmin, xmax-x, y-ymin, ymax-end_y]
    for (cx, cy), (sx, sy) in boxes:
        dx = max(cx-sx/2-x, 0., x-(cx+sx/2))
        dy = max(cy-sy/2-end_y, 0., y-(cy+sy/2))
        distances.append(math.hypot(dx, dy))
    if min(distances) + 1e-9 < SELECTION_CLEARANCE_M:
        raise ValueError('B/final approach has less than 2.4m static/parked-cart clearance; review the scene')
    return dict(goal=[x, y, YAW], navigation_approach=[x, end_y, YAW],
                selection_min_clearance_m=min(distances))


def parse_goal(text, source):
    data = yaml.load(text, Loader=UniqueKeysLoader)
    resolved = validate_selection(data)
    return dict(schema=1, frame='map', source=str(Path(source).resolve()),
                source_sha256=hashlib.sha256(text.encode()).hexdigest(), source_text=text,
                **resolved)


def load_goal(path=None):
    path = Path(path) if path is not None else default_goal_file()
    return parse_goal(path.read_bytes().decode('utf-8'), path)


def runtime_goal():
    """Simulator reads the orchestrator snapshot; standalone advanced launch reads YAML."""
    snapshot = os.environ.get(SNAPSHOT_ENV)
    if not snapshot:
        return load_goal()
    data = json.loads(Path(snapshot).read_text())
    checked = parse_goal(data['source_text'], data['source'])
    if data != checked:
        raise ValueError('Presentation goal snapshot is inconsistent; refusing a stale/mixed goal')
    return checked


if __name__ == '__main__':
    try:
        print(json.dumps(load_goal(), indent=2))
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise SystemExit(f'Invalid presentation/goal.yaml: {exc}')

"""Position-only task input. Never reads scene geometry or operator guided routes."""
import hashlib
import math
from pathlib import Path

import yaml

SNAPSHOT_ENV = 'T1_EXPLORATION_TASK_SNAPSHOT'


def load_task(path):
    text = Path(path).read_text()
    class Loader(yaml.SafeLoader):
        pass
    def mapping(loader, node):
        pairs = loader.construct_pairs(node, deep=True)
        result = dict(pairs)
        if len(result) != len(pairs):
            raise ValueError('duplicate task keys')
        return result
    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    data = yaml.load(text, Loader)
    if (not isinstance(data, dict) or set(data) != {'schema', 'frame', 'goal'}
            or type(data['schema']) is not int or data['schema'] != 1 or data['frame'] != 'start'
            or not isinstance(data['goal'], dict) or set(data['goal']) != {'x', 'y'}):
        raise ValueError('task requires schema: 1, frame: start, goal: {x, y}')
    xy = [data['goal'][key] for key in ('x', 'y')]
    if any(type(v) not in (float, int) or not math.isfinite(v) for v in xy):
        raise ValueError('goal must contain finite metre coordinates')
    return dict(schema=1, frame='start', offset=[float(v) for v in xy],
                source_sha256=hashlib.sha256(text.encode()).hexdigest())

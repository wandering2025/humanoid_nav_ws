"""Verify calibrated runtime files without requiring development Git history."""
import hashlib
import json
from pathlib import Path


def verify_runtime(workspace):
    root = Path(workspace)
    lock = json.loads((root / 'scripts/advanced/runtime-lock.json').read_text())
    if lock['schema'] != 1 or not lock['files']:
        raise RuntimeError('Invalid runtime contract manifest')
    changed = [name for name, expected in lock['files'].items()
               if not (root / name).is_file()
               or hashlib.sha256((root / name).read_bytes()).hexdigest() != expected]
    if changed:
        raise RuntimeError(f'Calibrated runtime files changed: {changed}. '
                           'Validate control changes before updating runtime-lock.json.')
    return lock['source_revision']

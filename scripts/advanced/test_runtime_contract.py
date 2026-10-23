"""A clean source export retains the calibrated-control contract."""
import hashlib
import json
from pathlib import Path

import pytest
from runtime_contract import verify_runtime


def test_current_runtime_matches_manifest():
    assert verify_runtime(Path(__file__).resolve().parents[2])


@pytest.mark.parametrize('mutation', ['unchanged', 'changed', 'missing'])
def test_contract_without_git_history(tmp_path, mutation):
    source = tmp_path / 'control.py'
    source.write_text('calibrated controller\n')
    manifest = tmp_path / 'scripts/advanced/runtime-lock.json'
    manifest.parent.mkdir(parents=True)
    manifest.write_text(json.dumps({'schema': 1, 'source_revision': 'recorded-revision',
        'files': {'control.py': hashlib.sha256(source.read_bytes()).hexdigest()}}))
    if mutation == 'changed':
        source.write_text('modified controller\n')
    elif mutation == 'missing':
        source.unlink()
    if mutation == 'unchanged':
        assert verify_runtime(tmp_path) == 'recorded-revision'
    else:
        with pytest.raises(RuntimeError, match='Calibrated runtime files changed'):
            verify_runtime(tmp_path)

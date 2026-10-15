import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import finalize


class Finalization(unittest.TestCase):
    def run_case(self,status,exit_code,expected,validation=True):
        directory=Path(tempfile.mkdtemp());p=directory/'result.json'
        p.write_text(json.dumps(dict(status=status,validation_before_port_check=validation)))
        original=p.read_bytes()
        with patch.object(finalize,'remaining_owned',return_value=[]),patch.object(finalize,'bindable',return_value=True):
            self.assertEqual(finalize.finish(directory,exit_code,0),expected)
        self.assertEqual(p.read_bytes(),original)
        return json.loads((directory/'final-result.json').read_text())

    def test_pending_success_is_finalized_after_runner_exit(self):
        self.assertEqual(self.run_case('PENDING_PORT_CHECK',4,0)['status'],'SUCCESS')

    def test_valid_navigation_failure_cannot_become_success(self):
        self.assertEqual(self.run_case('FAILED',1,1,False)['status'],'FAILED')

    def test_unexpected_runner_crash_cannot_become_success(self):
        self.assertEqual(self.run_case('PENDING_PORT_CHECK',-11,-11)['status'],'FAILED')

    def test_leftover_blocks_success_without_killing(self):
        d=Path(tempfile.mkdtemp());(d/'result.json').write_text('{"status":"PENDING_PORT_CHECK","validation_before_port_check":true}')
        with patch.object(finalize,'remaining_owned',return_value=[123]),patch.object(finalize,'bindable',return_value=True):
            self.assertEqual(finalize.finish(d,4,0),1)

if __name__=='__main__':unittest.main()

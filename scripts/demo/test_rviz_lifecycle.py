import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from gpu_watch import ERROR
from host_graphics import HostGraphics
from rviz_minimal import claim

class Lifecycle(unittest.TestCase):
    def test_minimal_attempt_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'attempt';claim(p,{'first':True})
            with self.assertRaises(FileExistsError):claim(p,{'second':True})
            self.assertEqual(json.loads(p.read_text()),{'first':True})

    def test_build_check_creates_no_display(self):
        binary=Path(__file__).resolve().parents[2]/'build/demo-rviz/t1_demo_rviz'
        r=subprocess.run([str(binary),'--check-build'],env={'PATH':'/usr/bin:/bin','DISPLAY':''},capture_output=True,text=True,timeout=5)
        self.assertEqual(r.returncode,0);self.assertIn('no display/context created',r.stdout)

    def test_new_windows_event_blocks(self):
        with tempfile.TemporaryDirectory() as d:
            host=HostGraphics.__new__(HostGraphics);host.out=Path(d);host.logs={};host.baseline={};host.checks=[]
            with patch('host_graphics.events',return_value={'Events':[{'Id':141}]}):
                with self.assertRaisesRegex(RuntimeError,'New Windows'):host.check(ERROR,windows=True)

    def test_only_new_wslg_error_is_relevant(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'weston';p.write_text('old Removing Device\n')
            host=HostGraphics.__new__(HostGraphics);host.out=Path(d);host.logs={p:p.stat().st_size}
            host.check(ERROR)
            with p.open('a') as f:f.write('new DX12_PAGEFAULT\n')
            with self.assertRaisesRegex(RuntimeError,'New WSLg'):host.check(ERROR)

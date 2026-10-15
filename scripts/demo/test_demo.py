"""Meaningful process-isolation and port-error regressions; no ROS motion."""
import errno
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('T1_DEMO_DIR', tempfile.mkdtemp(prefix='t1-demo-test-'))
os.environ.setdefault('T1_DEMO_RUN_ID','test')
spec=importlib.util.spec_from_file_location('demo',Path(__file__).with_name('demo.py'))
demo=importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


class Isolation(unittest.TestCase):
    def test_pid_reuse_guard(self):
        own=demo.identity(os.getpid())
        self.assertTrue(demo.alive(own))
        self.assertFalse(demo.alive(dict(own,start=str(int(own['start'])+1))))

    def test_track_excludes_unrelated_process(self):
        child=subprocess.Popen(['sleep','10'])
        unrelated=subprocess.Popen(['sleep','10'])
        try:
            instance=object.__new__(demo.Demo)
            instance.children={'goal':(child,None)}
            instance.owned={}
            instance.track()
            self.assertIn(child.pid,instance.owned)
            self.assertNotIn(unrelated.pid,instance.owned)
            self.assertEqual(instance.owned[child.pid]['owner'],'goal')
        finally:
            child.terminate();child.wait()
            unrelated.terminate();unrelated.wait()

    def test_permission_error_not_reported_as_occupied_port(self):
        with patch.object(demo.socket,'socket',side_effect=PermissionError(errno.EACCES,'restricted')):
            with self.assertRaisesRegex(RuntimeError,'Cannot inspect DDS port'):
                demo.port_free()

    def test_help_needs_no_environment(self):
        result=subprocess.run([str(demo.WS/'scripts/run_demo.sh'),'--help'],env={'PATH':'/usr/bin:/bin'},
                              cwd='/tmp',capture_output=True,text=True)
        self.assertEqual(result.returncode,0)
        self.assertIn('--preflight',result.stdout)


if __name__=='__main__':
    unittest.main()

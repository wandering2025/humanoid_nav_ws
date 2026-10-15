"""CPU-only safety contract. Never imports graphics/CUDA or opens GPU nodes."""
import json
from pathlib import Path
import subprocess
import sys
import unittest

HERE=Path(__file__).resolve().parent


class Guard(unittest.TestCase):
    def child(self, body):
        return subprocess.run([sys.executable,'-c',
            "import sys; sys.path.insert(0,sys.argv[1]); import cpu_guard; report=cpu_guard.enter(); "+body,
            str(HERE)],capture_output=True,text=True,timeout=10)

    def test_harmless_device_denied(self):
        child=self.child("import json; print(json.dumps(report))")
        self.assertEqual(child.returncode,0,child.stderr)
        report=json.loads(child.stdout)
        self.assertGreaterEqual(report['landlock_abi'],5)
        self.assertEqual(report['forbidden_probe_errno'],13)
        self.assertEqual(report['no_new_privs'],1)
        self.assertNotIn('/dev',report['allowed_paths'])
        self.assertNotIn('/mnt',report['allowed_paths'])
        self.assertFalse(report['cuda_probe'])

    def test_exec_inherits_denial_and_renderer_disabled(self):
        body="""import subprocess,os; p=subprocess.run([sys.executable,'-c',
'import os,errno; assert not os.environ["DISPLAY"]; assert not os.environ["CUDA_VISIBLE_DEVICES"]; '
'\\ntry: os.open("/dev/full",os.O_RDONLY)\\nexcept OSError as e: assert e.errno==errno.EACCES\\nelse: raise AssertionError("device allowed")']); sys.exit(p.returncode)"""
        child=self.child(body)
        self.assertEqual(child.returncode,0,child.stderr)

    def test_cpu_files_and_working_directory_are_usable(self):
        child=self.child("from pathlib import Path; import tempfile; p=Path(tempfile.mkdtemp())/'state'; p.write_text('cpu'); assert p.read_text()=='cpu'; p.unlink()")
        self.assertEqual(child.returncode,0,child.stderr)


if __name__=='__main__':
    unittest.main()

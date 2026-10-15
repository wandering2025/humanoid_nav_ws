"""Offline regression tests against preserved rehearsal evidence; no rendering."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import finalize
from dds_teardown import assess

ROOT=Path(__file__).resolve().parents[2]


def fixture():
    # The original remains FAILED; only the in-memory assessment gains explicit
    # lifecycle proof. Runtime now records this observation before sending SIGINT.
    raw=json.loads((ROOT/'docs/evidence/W14/rviz-lifecycle/full-02-final-result.json').read_text())
    raw['dds_shutdown']=dict(pid=45795,alive_before_signal=True,
        ros_communication_healthy=True,navigation_and_stop_complete=True,signal='SIGINT')
    proof=dict(dds_log_saved=True,dds_log_sha256='a'*64,dds_returncode=-11,
        dds_pid_gone=True,dds_port_released=True,remaining_owned_pids=[],
        remaining_project_processes=[],host_graphics_checked_without_fault=True,runner_exit_code=1)
    return raw,proof


class ExceptionClassification(unittest.TestCase):
    def fails(self,raw,proof):
        result=assess(raw,proof)
        self.assertEqual(result['status'],'FAILED');self.assertTrue(result['failed_checks'])

    def test_success_stop_all_other_zero_is_success_with_warning(self):
        raw,proof=fixture();result=assess(raw,proof)
        self.assertEqual(result['status'],'SUCCESS_WITH_WARNING')
        self.assertEqual(result['dds_returncode'],-11);self.assertEqual(result['failed_checks'],[])
        self.assertEqual(raw['status'],'FAILED')

    def test_dds_dies_during_navigation_fails(self):
        raw,proof=fixture();raw['dds_shutdown']['alive_before_signal']=False
        self.fails(raw,proof)

    def test_navigation_failure_and_dds_segfault_fails(self):
        raw,proof=fixture();raw['arrived']=False;raw['nav2']['status']='ABORTED'
        self.fails(raw,proof)

    def test_safety_stop_failure_and_dds_segfault_fails(self):
        for field in ('ack','physical_stationary'):
            with self.subTest(field=field):
                raw,proof=fixture();raw['safe_stop'][field]=False;self.fails(raw,proof)

    def test_rviz_mujoco_launch_or_starter_nonzero_fails(self):
        for component in ('rviz','launch','t1_sim-1','navigation_starter-16'):
            with self.subTest(component=component):
                raw,proof=fixture()
                if component in raw['parent_exit_codes']:raw['parent_exit_codes'][component]=-6
                else:raw['launch_process_exit_codes'][component]['exit_code']=-6
                self.fails(raw,proof)

    def test_communication_interruption_fails(self):
        raw,proof=fixture();raw['dds_shutdown']['ros_communication_healthy']=False
        self.fails(raw,proof)

    def test_missing_or_unexpected_shutdown_signal_fails(self):
        for value in ([],[['dds',45795,'SIGTERM']]):
            raw,proof=fixture();raw['shutdown_signals']=value;self.fails(raw,proof)

    def test_signal_escalation_any_component_fails(self):
        for signal in ('SIGTERM','SIGKILL'):
            raw,proof=fixture();raw['shutdown_signals'].append(['launch',45821,signal]);self.fails(raw,proof)

    def test_missing_artifact_host_port_or_pid_proof_fails(self):
        for field in ('dds_log_saved','dds_log_sha256','dds_returncode','dds_pid_gone',
                      'dds_port_released','host_graphics_checked_without_fault'):
            with self.subTest(field=field):
                raw,proof=fixture();proof.pop(field);self.fails(raw,proof)

    def test_live_residual_or_missing_robot_component_fails(self):
        raw,proof=fixture();proof['remaining_owned_pids']=[45795];self.fails(raw,proof)
        raw,proof=fixture();proof['remaining_project_processes']=[{'pid':123}];self.fails(raw,proof)
        raw,proof=fixture();raw['launch_process_exit_codes'].pop('t1_sim-1');self.fails(raw,proof)

    def test_host_fault_or_any_other_error_fails(self):
        for field in ('gpu_monitor_error','error','cleanup_error','metric_error','graph_error'):
            raw,proof=fixture();raw[field]='observed fault';self.fails(raw,proof)

    def test_collision_fall_or_dynamic_failure_fails(self):
        raw,proof=fixture();raw['contacts']['obstacle']=1;self.fails(raw,proof)
        raw,proof=fixture();raw['fell']=True;self.fails(raw,proof)
        raw,proof=fixture();raw['dynamic']['verdict']='FAIL';self.fails(raw,proof)

    def test_unexpected_runner_crash_fails(self):
        raw,proof=fixture();proof['runner_exit_code']=-11;self.fails(raw,proof)

    def test_warning_is_visible_wrapper_success_but_dds_exit_stays_minus11(self):
        raw,proof=fixture()
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);path=directory/'result.json'
            path.write_text(json.dumps(raw));original=path.read_bytes();output=io.StringIO()
            with patch.object(finalize,'remaining_owned',return_value=[]), \
                 patch.object(finalize,'bindable',return_value=True), \
                 patch.object(finalize,'evidence',return_value=proof), \
                 contextlib.redirect_stderr(output):
                self.assertEqual(finalize.finish(directory,1,0),0)
            result=json.loads((directory/'final-result.json').read_text())
            self.assertEqual(result['status'],'SUCCESS_WITH_WARNING')
            self.assertEqual(result['dds_returncode'],-11)
            self.assertEqual(result['parent_exit_codes']['dds'],-11)
            self.assertIn('DDS_TEARDOWN_WARNING dds_returncode=-11',output.getvalue())
            self.assertEqual(path.read_bytes(),original)

    def test_finalizer_rejects_incomplete_dds_log_and_live_dds_pid(self):
        import os
        raw,unused=fixture()
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);log=directory/'dds.log'
            log.write_text('### Server is running ###\n')
            with patch.object(finalize,'project_processes',return_value=[]), \
                 patch.object(finalize,'graphics_proof',return_value=True):
                proof=finalize.evidence(directory,raw,1,[],True)
                self.assertFalse(proof['dds_log_saved']);self.fails(raw,proof)
                log.write_text('### Server is running ###\n### Server shut down ###\n')
                raw['dds_shutdown']['pid']=os.getpid()
                proof=finalize.evidence(directory,raw,1,[],True)
                self.assertTrue(proof['dds_log_saved']);self.assertFalse(proof['dds_pid_gone'])
                self.fails(raw,proof)

    def test_existing_rehearsals_assessed_without_rewriting_failed_original(self):
        from assess_release_evidence import build
        path=ROOT/'docs/evidence/W14/rviz-lifecycle/full-02-final-result.json'
        original=path.read_bytes();result=build()
        self.assertEqual([r['publication_status'] for r in result['rehearsals']],
                         ['SUCCESS','SUCCESS_WITH_WARNING'])
        self.assertEqual(result['rehearsals'][1]['original_status'],'FAILED')
        self.assertEqual(path.read_bytes(),original)

    def test_new_graphics_halt_preserves_dds_exception_and_original_canaries(self):
        import hashlib
        profile=json.loads((ROOT/'scripts/demo/gpu-profile.json').read_text())
        self.assertEqual(profile['status'],'GPU_HALTED_W15_NVIDIA_VIEWER_EXIT')
        self.assertEqual(profile['release_status'],'FINAL_RELEASE_BLOCKED')
        blocker=profile['w15_blocker']
        self.assertEqual(hashlib.sha256((ROOT/blocker['path']).read_bytes()).hexdigest(),blocker['sha256'])
        halt=profile['halt_evidence']
        self.assertEqual(hashlib.sha256((ROOT/halt['path']).read_bytes()).hexdigest(),halt['sha256'])
        for name,expected in profile['stages'].items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),expected)

    def test_saved_graphics_fault_or_missing_checks_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);host=root/'gpu/host';host.mkdir(parents=True)
            self.assertFalse(finalize.graphics_proof(root))
            (host/'windows-baseline.json').write_text('{}')
            (host/'windows-checks.json').write_text('[{"Events":[]}]')
            self.assertTrue(finalize.graphics_proof(root))
            (host/'stderr.log.delta').write_text('D3D12 Removing Device')
            self.assertFalse(finalize.graphics_proof(root))

if __name__=='__main__':unittest.main()

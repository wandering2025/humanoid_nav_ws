"""Meaningful process-isolation and port-error regressions; no ROS motion."""
import errno
import importlib.util
import io
import os
from pathlib import Path
import subprocess
import tempfile
import time
import shutil
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault('T1_DEMO_DIR', tempfile.mkdtemp(prefix='t1-demo-test-'))
os.environ.setdefault('T1_DEMO_RUN_ID','test')
spec=importlib.util.spec_from_file_location('demo',Path(__file__).with_name('demo.py'))
demo=importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


class Isolation(unittest.TestCase):
    def test_first_clock_absence_is_startup_not_frozen_clock(self):
        self.assertFalse(demo.clock_stalled(SimpleNamespace(first_clock=None,sim=0.),20.,0.))
        self.assertFalse(demo.clock_stalled(SimpleNamespace(first_clock=0.,sim=0.),20.,0.))
        self.assertTrue(demo.clock_stalled(SimpleNamespace(first_clock=0.,sim=.02),20.,16.9))
        self.assertFalse(demo.clock_stalled(SimpleNamespace(first_clock=0.,sim=.02),20.,17.1))

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

    def test_cleanup_reaps_cooperative_child_and_preserves_unrelated(self):
        instance=object.__new__(demo.Demo)
        instance.children={}; instance.owned={}; instance.node=None
        instance.cleanup_started=False; instance.summary={}; instance.started=time.monotonic()
        instance.launch_log=demo.OUT/'absent-launch.log'
        unrelated=subprocess.Popen(['sleep','30'])
        script="import signal,time,sys; signal.signal(signal.SIGINT,lambda *_:sys.exit(0)); print('READY',flush=True); time.sleep(30)"
        child=instance.spawn('goal',[demo.sys.executable,'-c',script])
        try:
            deadline=time.monotonic()+5
            while 'READY' not in (demo.OUT/'goal.log').read_text():
                self.assertLess(time.monotonic(),deadline)
                time.sleep(.01)
            with patch.object(demo,'port_free',return_value=True), patch.object(demo,'port_connections',return_value={}):
                instance.cleanup()
            self.assertEqual(child.returncode,0)
            self.assertEqual(instance.summary['remaining_owned_pids'],[])
            self.assertIsNone(unrelated.poll())
        finally:
            if child.poll() is None:child.kill();child.wait()
            unrelated.terminate();unrelated.wait()

    def test_cleanup_keeps_early_dds_segfault_and_does_not_claim_expected_signal(self):
        instance=object.__new__(demo.Demo)
        child=SimpleNamespace(pid=987654321,poll=lambda:-11,wait=lambda timeout:-11)
        instance.children={'dds':(child,io.StringIO())};instance.owned={};instance.node=None
        instance.cleanup_started=False;instance.summary={};instance.started=time.monotonic()
        instance.launch_log=demo.OUT/'absent-launch.log'
        with patch.object(instance,'track'),patch.object(demo,'project_processes',return_value=[]), \
             patch.object(demo,'port_connections',return_value={}):
            instance.cleanup()
        self.assertEqual(instance.summary['dds_returncode'],-11)
        self.assertEqual(instance.summary['parent_exit_codes']['dds'],-11)
        self.assertFalse(instance.summary['dds_shutdown']['alive_before_signal'])
        self.assertIsNone(instance.summary['dds_shutdown']['signal'])
        self.assertEqual(instance.summary['shutdown_signals'],[])

    def test_permission_error_not_reported_as_occupied_port(self):
        with patch.object(demo.socket,'socket',side_effect=PermissionError(errno.EACCES,'restricted')):
            with self.assertRaisesRegex(RuntimeError,'Cannot inspect DDS port'):
                demo.port_free()

    def test_port_binding_is_deferred_until_runner_exit(self):
        state={'node_closed':False,'context_closed':False}
        instance=object.__new__(demo.Demo)
        instance.children={};instance.owned={};instance.cleanup_started=False
        instance.summary={};instance.started=time.monotonic();instance.runtime_graph={}
        instance.launch_log=demo.OUT/'absent-launch.log'
        instance.node=SimpleNamespace(events=[],initializations=[],clock_regressions=0,
            destroy_node=lambda:state.update(node_closed=True))
        fake=SimpleNamespace(ok=lambda:True,shutdown=lambda:state.update(context_closed=True))
        def bind_check():
            raise AssertionError('port bind must happen only in the independent finalizer')
        with patch.dict(demo.sys.modules,{'rclpy':fake}),patch.object(demo,'port_free',side_effect=bind_check),patch.object(demo,'port_connections',return_value={}):
            instance.cleanup()
        self.assertTrue(state['node_closed']);self.assertTrue(state['context_closed'])
        self.assertTrue(instance.summary['dds_port_check_deferred'])

    def test_time_wait_waits_without_signalling_any_process(self):
        with patch.object(demo,'port_free',side_effect=[False,True]), \
             patch.object(demo,'port_connections',return_value={'listeners':[],'time_wait':[{}],'other_connections':[]}), \
             patch.object(demo.time,'sleep'), patch.object(demo.os,'kill',side_effect=AssertionError('must not kill')):
            self.assertTrue(demo.wait_port_reusable(.5))

    def test_observer_first_shutdown_is_opt_in_and_retains_disconnect_errors(self):
        for enabled,blocked in ((False,False),(True,False),(True,True)):
            with self.subTest(enabled=enabled,blocked=blocked):
                events=[];state={'closed':False,'code':None,'now':0.}
                instance=object.__new__(demo.Demo)
                instance.close_observer_before_dds=enabled
                instance.owned={};instance.cleanup_started=False
                instance.summary={};instance.started=0.;instance.runtime_graph={}
                instance.launch_log=demo.OUT/'absent-launch.log'
                instance.node=SimpleNamespace(events=['saved evidence'],initializations=[],clock_regressions=0,
                    destroy_node=lambda:events.append('node_closed'))
                def shutdown():
                    events.append('context_closed');state['closed']=True
                def signal_child(sig):
                    self.assertEqual(state['closed'],enabled)
                    events.append('dds_signal');state['code']=0
                child=SimpleNamespace(pid=987654321,poll=lambda:state['code'],
                    wait=lambda timeout:state['code'],send_signal=signal_child)
                instance.children={'dds':(child,io.StringIO())}
                fake=SimpleNamespace(ok=lambda:not state['closed'],shutdown=shutdown)
                connections=dict(listeners=[{}],time_wait=[],other_connections=[{}] if blocked else [])
                with patch.dict(demo.sys.modules,{'rclpy':fake}),patch.object(instance,'track'), \
                     patch.object(demo,'project_processes',return_value=[]), \
                     patch.object(demo,'port_connections',return_value=connections), \
                     patch.object(demo.time,'monotonic',side_effect=lambda:state['now']), \
                     patch.object(demo.time,'sleep',side_effect=lambda dt:state.update(now=state['now']+dt)):
                    instance.cleanup();instance.close_observer()
                expected=['node_closed','context_closed','dds_signal'] if enabled else ['dds_signal','node_closed','context_closed']
                self.assertEqual(events,expected)
                self.assertEqual(instance.summary['events'],['saved evidence'])
                self.assertEqual(instance.summary['parent_exit_codes']['dds'],0)
                self.assertEqual(bool(instance.summary.get('observer_shutdown_error')),blocked)
                if enabled:
                    self.assertEqual(instance.summary['dds_shutdown']['clients_disconnected'],not blocked)

    def test_foreign_listener_is_never_cleaned_up(self):
        with patch.object(demo,'port_free',return_value=False), \
             patch.object(demo,'port_connections',return_value={'listeners':[{}],'time_wait':[],'other_connections':[]}), \
             patch.object(demo.os,'kill',side_effect=AssertionError('must not kill')):
            self.assertFalse(demo.wait_port_reusable(.5))

    def test_help_needs_no_environment(self):
        result=subprocess.run([str(demo.WS/'scripts/run_demo.sh'),'--help'],env={'PATH':'/usr/bin:/bin'},
                              cwd='/tmp',capture_output=True,text=True)
        self.assertEqual(result.returncode,0)
        self.assertIn('--preflight',result.stdout)

    def test_all_public_runtime_modes_stop_before_environment(self):
        isolated=Path(tempfile.mkdtemp())/'scripts'
        isolated.mkdir();entry=isolated/'run_demo.sh'
        shutil.copyfile(demo.WS/'scripts/run_demo.sh',entry);entry.chmod(0o755)
        for arguments in ([],['--no-rviz'],['--scenario','U01']):
            result=subprocess.run([str(entry),*arguments],
                                  env={'PATH':'/usr/bin:/bin'},cwd='/tmp',capture_output=True,text=True)
            self.assertEqual(result.returncode,3,arguments)
            self.assertIn('W14_BLOCKED',result.stderr)

    def test_direct_runner_cannot_probe_gpu_while_locked(self):
        with patch.object(demo,'GPU_PROFILE',Path('/tmp/t1-test-missing-profile.json')), patch.object(demo.sys,'argv',['demo.py']), patch.object(demo,'preflight',side_effect=AssertionError('must not execute')):
            self.assertEqual(demo.main(),3)

    def test_heap_halt_rejects_before_creating_gpu_watch(self):
        import json
        with tempfile.TemporaryDirectory() as directory:
            profile=Path(directory)/'profile.json'
            profile.write_text(json.dumps(dict(status='GPU_HALTED_RVIZ_HEAP_CORRUPTION',stages={})))
            with patch.object(demo,'GPU_PROFILE',profile), patch.object(demo.sys,'argv',['demo.py']), patch('gpu_watch.Watch',side_effect=AssertionError('must not touch GPU')):
                self.assertEqual(demo.main(),3)


if __name__=='__main__':
    unittest.main()


def test_heartbeat_uses_monotonic_progress_despite_wall_clock_steps():
    heartbeat_progress=demo.heartbeat_progress
    state,stalled=heartbeat_progress(None,1000000000,10.)
    assert not stalled
    # Same timestamp during a +2.78s wall jump: only .3 monotonic seconds elapsed.
    state,stalled=heartbeat_progress(state,1000000000,10.3)
    assert not stalled
    state,stalled=heartbeat_progress(state,4000000000,10.4)
    assert not stalled
    _,stalled=heartbeat_progress(state,4000000000,13.41)
    assert stalled
    # A backwards wall step still counts as a fresh write.
    _,stalled=heartbeat_progress(state,2000000000,13.41)
    assert not stalled

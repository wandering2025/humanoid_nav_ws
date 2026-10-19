"""Exercise the actual endpoint matcher with delayed exact-stamp TF delivery."""
from pathlib import Path
from types import MethodType, SimpleNamespace

def test_delayed_tf_preserves_raw_scan_timestamp_hits_and_original_cm_state(tmp_path,monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[2]))
    monkeypatch.setenv('T1_WS',str(Path(__file__).parents[3]))
    monkeypatch.setenv('T1_EVAL_OUT',str(tmp_path))
    from evaluation.harness import Probe
    from evaluation.scan_evidence import DeferredSamples
    calls=[]
    ready=False
    def lookup(parent,child,stamp):
        calls.append((parent,child,stamp.nanoseconds))
        if not ready:
            raise LookupError('scan arrived before corresponding TF')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.,z=1.),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    scan=SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=7,nanosec=100000000),frame_id='laser_stabilized'),
        angle_min=0.,angle_increment=.1,ranges=[2.,float('inf')])
    obstacle=dict(position=[2.,0.,.75],size=[.5,.5,1.5],phase='hold_0')
    queue=DeferredSamples()
    node=SimpleNamespace(dynamic_pending=queue,match_obstacle=lambda t:obstacle,
        buffer=SimpleNamespace(lookup_transform=lookup),dynamic_scans=[],cm=(0,''))
    node.sample_dynamic_scan=MethodType(Probe.sample_dynamic_scan,node)
    queue.add((scan,(1,'Stop')),10.)
    queue.drain(lambda item:node.sample_dynamic_scan(*item),10.01)
    assert not node.dynamic_scans and queue.stats['missing_tf_attempts']==1
    ready=True
    queue.drain(lambda item:node.sample_dynamic_scan(*item),10.10)
    queue.drain(lambda item:node.sample_dynamic_scan(*item),10.11)
    assert calls==[('odom','laser_stabilized',7100000000)]*2
    assert len(node.dynamic_scans)==1
    result=node.dynamic_scans[0]
    assert abs(result['sim']-7.1)<1e-8
    assert result['hits']==[[0,2.,2.,0.]] and result['cm']==(1,'Stop')

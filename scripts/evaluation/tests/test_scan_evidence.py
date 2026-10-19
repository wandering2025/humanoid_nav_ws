from pathlib import Path
import importlib.util

spec=importlib.util.spec_from_file_location('scan_evidence',Path(__file__).parents[1]/'scan_evidence.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
DeferredSamples=module.DeferredSamples


def test_delayed_exact_evidence_is_sampled_once_without_replacing_input():
    queue=DeferredSamples()
    raw=object();context=object();item=(raw,context)
    queue.add(item,10.)
    queue.drain(lambda candidate:False,10.01)  # exact TF/obstacle not ready
    assert len(queue.pending)==1
    accepted=[]
    queue.drain(lambda candidate:accepted.append(candidate) or True,10.1)
    queue.drain(lambda candidate:accepted.append(candidate) or True,10.2)
    assert accepted==[item] and accepted[0][0] is raw and accepted[0][1] is context
    assert queue.summary()['sampled']==1 and queue.summary()['pending']==0


def test_unavailable_timestamps_expire_without_latest_transform_fallback():
    queue=DeferredSamples(wait_s=.5)
    queue.add('no exact data',1.)
    queue.drain(lambda candidate:False,1.3)
    def too_late(candidate):
        raise AssertionError('expired input must not be evaluated with newer data')
    queue.drain(too_late,1.51)
    assert queue.summary()['expired']==1 and queue.summary()['sampled']==0


def test_backlog_is_bounded_and_a_missing_sample_does_not_block_ready_ones():
    queue=DeferredSamples(capacity=2)
    for item in ('overflow','missing','ready'):
        queue.add(item,0.)
    queue.drain(lambda candidate:candidate=='ready',.1)
    summary=queue.summary()
    assert summary['overflow']==1 and summary['sampled']==1 and summary['pending']==1
    assert summary['max_pending']==2

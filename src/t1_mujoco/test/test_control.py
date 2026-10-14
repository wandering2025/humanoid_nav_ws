from t1_mujoco.control import InputBuffer


def ready_buffer():
    b = InputBuffer()
    b.boundary('READY', 10.)
    b.receive((.5,0,0,0,0,.2), wall=20.)
    return b


def test_dual_timeouts_and_no_minimum_boost():
    b=ready_buffer()
    assert b.consume(10.1,20.1,.5,.5,True)[1] == (.5,0,.2)
    assert b.consume(10.6,20.1,.5,.5,True)[2] == 'TIMEOUT'
    assert b.consume(10.1,20.6,.5,.5,True)[2] == 'TIMEOUT'
    b.receive((.01,0,0,0,0,0),wall=20.)
    assert b.consume(10.1,20.1,.5,.5,True)[1][0] == .01


def test_all_six_components_checked_and_recoverable():
    for i in range(6):
        for invalid in (float('nan'),float('inf'),-float('inf')):
            b=ready_buffer(); values=[0.]*6;values[i]=invalid
            b.receive(values,wall=20.)
            assert b.consume(10.,20.,.5,.5,True)[1:] == ((0.,0.,0.),'INVALID')
            b.receive((.5,0,0,0,0,0),wall=20.)
            assert b.consume(10.,20.,.5,.5,True)[2]=='ACTIVE'


def test_pause_clears_and_discards_commands():
    b=ready_buffer()
    b.boundary('PAUSED',10.,clear=True)
    b.receive((1.,0,0,0,0,0),wall=20.)
    b.boundary('READY',10.,clear=True)
    assert b.consume(10.,20.,.5,.5,True)[1:] == ((0.,0.,0.),'IDLE')


def test_uncalibrated_and_upstream_debug_clamp():
    b=ready_buffer()
    assert b.consume(10.,20.,.5,.5,False)[1:] == ((0.,0.,0.),'UNCALIBRATED')
    b.receive((5.,-5.,1.,1.,1.,5.),wall=20.)
    assert b.consume(10.,20.,.5,.5,True)[1] == (1.,-1.,1.)
    assert b.ignored_components==1


def test_safety_lease_veto_and_no_cached_revival():
    b = ready_buffer()
    b.safety_required = True
    assert b.consume(10.,20.,.5,.5,True)[2] == 'BLOCKED_BY_STATE'
    b.safety(True,wall=20.)
    b.receive((.5,0,0,0,0,0),wall=20.)
    assert b.consume(10.,20.1,.5,.5,True)[1] == (.5,0.,0.)
    # Permit timeout independent of cmd timeout; monotonic even if sim frozen.
    assert b.consume(10.,20.16,.5,.5,True)[2] == 'BLOCKED_BY_STATE'
    b.safety(True,wall=20.17)
    assert b.consume(10.,20.17,.5,.5,True)[1] == (0.,0.,0.)
    b.receive((.5,0,0,0,0,0),wall=20.18)
    b.safety(False,wall=20.19)
    b.safety(True,wall=20.20)
    assert b.consume(10.,20.20,.5,.5,True)[1] == (0.,0.,0.)
    assert b.consume(10.2,20.21,.5,.5,True)[2] == 'BLOCKED_BY_STATE'

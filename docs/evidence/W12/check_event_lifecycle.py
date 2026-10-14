"""Live C1 fixture: actual walking trigger, pause freezes, reset restages events."""
import json
import os
import time
from std_srvs.srv import SetBool, Trigger
from harness import OUT, WS, Run, Session

session = Session('event_lifecycle')
run, result = None, {}
try:
    run = Run('simulation.launch.py','event_lifecycle',[
        'scene:=dynamic_crossing_world',f'events_file:={WS}/src/t1_bringup/config/w12_events.yaml',
        'scenario:=blocking','max_sim_time:=60'],wall_limit=150)
    run.ready()
    run.arm_teleop()
    run.source = (.45,0.)
    run.until(lambda: run.node.obstacle and run.node.obstacle['phase']=='move_0',30)
    run.duration(1.)
    run.source = (0.,0.)
    response = run.call(SetBool,'/t1/sim/pause',SetBool.Request(data=True))
    assert response.success,response.message
    until = time.monotonic()+.5
    while time.monotonic()<until:run.spin()
    before = dict(run.node.obstacle)
    sim_before = run.node.sim
    until = time.monotonic()+1.
    while time.monotonic()<until:run.spin()
    after = dict(run.node.obstacle)
    result['pause'] = dict(before=before,after=after,sim_before=sim_before,sim_after=run.node.sim)
    response = run.call(Trigger,'/t1/sim/reset',Trigger.Request())
    assert response.success,response.message
    run.until(lambda:run.node.obstacle and run.node.obstacle['episode']==1,10)
    reset = dict(run.node.obstacle)
    run.duration(3.)
    result['reset'] = reset
    result['checks'] = dict(paused_clock=run.node.sim>sim_before and result['pause']['sim_after']==sim_before,
                           paused_position=before['position']==after['position'],
                           reset_trigger=reset['triggered_at'] is None,reset_path=reset['position']==[4.5,3.,.75],
                           reset_episode=reset['episode']==1,monotonic_clock=run.node.clock_regressions==0,
                           warmup_ready=run.node.state['state']=='READY')
except Exception as exc:
    result['error']=repr(exc)
finally:
    if run:
        closed=run.close()
        result['run']=closed
        result.setdefault('checks',{})['clean_exit']=closed['exit_code']==0 and closed['manifest'].get('exit_code')==0
        result['checks']['no_contacts']=not any((closed['manifest'].get('contact_counts') or {'missing':1}).values())
    session.close()
    result['verdict']='PASS' if result.get('checks') and all(result['checks'].values()) and not result.get('error') else 'FAIL'
    (OUT/os.environ.get('W12_EVENT_OUTPUT','c1-event-lifecycle.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result.get('checks')),result['verdict'],result.get('error'),flush=True)
raise SystemExit(0 if result['verdict']=='PASS' else 1)

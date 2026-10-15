"""Measured response predicates; neither this module nor truth drives navigation."""
import math

def polyline_gap(point,polyline):
    best=math.inf
    for a,b in zip(polyline,polyline[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];d=dx*dx+dy*dy
        t=max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/d)) if d else 0.
        best=min(best,math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy))
    return best

def replan_evidence(rows,plans,obstacles,navigation_start):
    active=[o for o in obstacles if o['phase'] not in ('staged','finished')]
    result=dict(active_motion_m=0.,plan_deviation_m=0.,robot_deviation_m=0.,replanned_while_active=False)
    if not active:return result
    a,b=active[0]['sim_time'],active[-1]['sim_time']
    previous=[p for p in plans if navigation_start<=p['sim']<a]
    current=[p for p in plans if a<=p['sim']<=b]
    samples=[r for r in rows if a<=r['sim']<=b and r['mx'] is not None]
    if not previous or not current or not samples:return result
    baseline=previous[-1]['points']
    result.update(active_interval=[a,b],baseline_plan_sim=previous[-1]['sim'],
        active_motion_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(samples,samples[1:])),
        plan_deviation_m=max(polyline_gap(pt,baseline) for p in current for pt in p['points']),
        robot_deviation_m=max(polyline_gap((r['mx'],r['my']),baseline) for r in samples))
    result['replanned_while_active']=bool(result['plan_deviation_m']>=.5 and result['robot_deviation_m']>=.25 and result['active_motion_m']>=.5)
    return result

def response_verdict(dynamic,arrived):
    sensor=dynamic['active_scan_hit_frames']>0 and dynamic['active_costmap_marked_frames']>0
    wait=dynamic['protective_stationary_s']>=.5
    replan=dynamic['replan']['replanned_while_active']
    ok=bool(sensor and dynamic['triggered'] and dynamic['finished'] and arrived and (wait or replan))
    return dict(verdict='PASS' if ok else 'FAIL',
                response=('PROTECTIVE_WAIT_AND_CONTINUE' if wait else 'REPLAN_AND_CONTINUE') if ok else 'UNVERIFIED')

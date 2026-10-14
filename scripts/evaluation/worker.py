"""One fresh ROS context and full process tree per episode."""
import json
import os
from pathlib import Path
import traceback

import yaml

from . import harness
from .episode import episode
from .metrics import body_clearance, classify, distribution, stopping_metrics, trace_columns


def run(case, suite, output, run_id, gui=False):
    session = harness.Session(run_id)
    result = None
    try:
        result = episode(case['id'], case['start'], case['goal'], suite['criteria'],
                         suite['criteria']['time_limit_s'], seed=case['seed'],
                         rviz=gui, post_s=12.)
    except Exception:
        result = dict(outcome='FAIL_STARTUP', error=traceback.format_exc())
    finally:
        for live in harness.ACTIVE_RUNS:
            if live.closed_result is None:
                closed = live.close(settle=0.)
                if result is not None:
                    result['run'] = closed
        session.close()
    result.update(case_spec=case, run_id=run_id, protocol='w13_v1')
    manifest = result.get('run', {}).get('manifest', {})
    dirty = manifest.get('dirty_files', [])
    allowed = dirty == [' M docs/start_commands.md']
    # git().strip() removes leading whitespace from the first status line in old RunLog.
    allowed = allowed or dirty == ['M docs/start_commands.md']
    checks = dict(source_clean=manifest.get('dirty') is False or allowed,
                  user_document_unchanged=harness.digest(harness.WS/'docs/start_commands.md') ==
                    '2cbea0e2865a633ada3df75db27a06464fe0e2c280e2971819aede1bbbaab0d5',
                  fresh_clock=result.get('run', {}).get('first_clock', 1e9) is not None and
                    (result.get('run', {}).get('first_clock') or 0.) < 5.,
                  monotonic_clock=result.get('run', {}).get('clock_regressions') == 0,
                  one_initialpose=len(result.get('run', {}).get('initializations', [])) == 1,
                  complete_manifest=all(manifest.get(k) is not None for k in
                    ('code_sha', 'checkpoint_sha256', 'mjcf_sha256', 'scene_sha256', 'map_sha256', 'parameters_sha256', 'initial_pose')),
                  frozen_timing=(manifest.get('physics_dt'),manifest.get('decimation'),manifest.get('policy_dt')) == (.002,10,.02))
    result['validity_checks'] = checks
    result['documentation_dirty_exception'] = dict(authorized=True, actual_dirty=manifest.get('dirty'),
        actual_dirty_files=dirty, document_sha256=harness.digest(harness.WS/'docs/start_commands.md'))
    artifact = harness.WS/'run_data'/run_id
    try:
        rows, trace = trace_columns(artifact/'sim_trace.csv')
        result['body_clearance'] = body_clearance(manifest, artifact/'geometry.jsonl')
        result['stopping'] = stopping_metrics(rows, trace, result['run']['events'])
        result['performance'] = {key: distribution(trace[key][trace['scan_due'] > 0] if key=='scan_compute_ms' else trace[key])
             for key in ('step_compute_ms', 'policy_compute_ms', 'physics_compute_ms', 'scan_compute_ms')}
        # Exclude the first 10 sim seconds so RTF has a full window after startup.
        result['performance']['rtf'] = distribution(trace['rtf_10s'][trace['sim_time'] >= 10])
        cpus = json.loads((output/f'{run_id}-cpu.json').read_text())
        result['performance']['navigation_cpu_ms_per_wall_s'] = {
             name: distribution([r['cpu_ms_per_wall_s'] for r in cpus if r['process']==name and
                                 r['sim'] > result.get('nav_active_sim', 1e9)])
             for name in sorted({r['process'] for r in cpus})}
        m = result.get('metrics', {})
        if not m.get('settle', {}).get('pos_err_m'):
            goal = case['goal']
            # Both frozen maps have identity T_world_map; apply it explicitly.
            T = manifest['T_world_map']; import math
            gx=T['x']+math.cos(T['yaw'])*goal[0]-math.sin(T['yaw'])*goal[1]
            gy=T['y']+math.sin(T['yaw'])*goal[0]+math.cos(T['yaw'])*goal[1]
            m['terminal_error'] = dict(pos_err_m=math.hypot(trace['x_w'][-1]-gx,trace['y_w'][-1]-gy),
                yaw_err_deg=math.degrees(abs(math.atan2(math.sin(trace['yaw_w'][-1]-goal[2]-T['yaw']), math.cos(trace['yaw_w'][-1]-goal[2]-T['yaw'])))))
        contact_file=artifact/'contact_events.csv'
        result['contact_events'] = list(__import__('csv').DictReader(contact_file.open()))
    except Exception:
        result['metric_error'] = traceback.format_exc()
        checks['offline_metrics_complete'] = False
    result['outcome_legacy'] = result.get('outcome')
    result['outcome'] = classify(result, all(checks.values()))
    # Independent exact config provenance, including evaluator and full scene/meta bytes.
    files = [harness.WS/'src/t1_bringup/config/test_suite_v1.yaml', harness.WS/suite['events_file'],
             harness.WS/f"src/t1_mujoco/worlds/{case['scene']}.yaml",
             harness.WS/f"src/t1_navigation/maps/{case['scene']}.meta.yaml"]
    result['inputs'] = {str(p.relative_to(harness.WS)): harness.digest(p) for p in files}
    result['manifest'] = dict(manifest, suite=case['suite'], case=case['id'], goal=case['goal'],
        scene_nominal_goal=manifest.get('goal'), tolerances=suite['criteria'],
        time_limit_s=suite['criteria']['time_limit_s'], simulator_manifest_sha256=harness.digest(artifact/'manifest.json') if (artifact/'manifest.json').exists() else None)
    (output/'result.json').write_text(json.dumps(result, indent=2, default=str)+'\n')
    print('W13_RESULT',run_id,result['outcome'],flush=True)
    return 0

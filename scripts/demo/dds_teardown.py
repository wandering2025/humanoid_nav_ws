"""Fail-closed release classification only; no ROS, GPU access or child signals."""
REQUIRED_COMPONENTS = {
    't1_sim', 'robot_state_publisher', 'map_server', 'amcl', 'command_gate',
    'velocity_smoother', 'collision_monitor', 'controller_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'navigation_adapter', 'navigation_starter',
    't1_initial_pose_once',
}


def assess(raw, proof):
    parents = raw.get('parent_exit_codes', {})
    components = raw.get('launch_process_exit_codes', {})
    names = {name.rsplit('-', 1)[0] for name in components}
    nav = raw.get('nav2', {})
    stop = raw.get('safe_stop', {})
    shutdown = raw.get('dds_shutdown', {})
    signals = raw.get('shutdown_signals', [])
    code = parents.get('dds')
    checks = {
        'navigation_success': raw.get('arrived') is True and nav.get('status') == 'SUCCEEDED'
            and bool(nav.get('verified')) and nav.get('exit') == 0
            and raw.get('dynamic', {}).get('verdict') == 'PASS',
        'zero_contacts_and_falls': raw.get('fell') is False and raw.get('contacts') ==
            dict(static_env=0, obstacle=0, ground_nonfoot=0),
        'safe_stop_completed': stop.get('ack') is True and stop.get('physical_stationary') is True
            and stop.get('gate', {}).get('state') == 'STOPPED',
        'other_components_zero': set(parents) == {'dds', 'launch', 'rviz', 'goal'}
            and all(parents.get(k) == 0 for k in ('launch', 'rviz', 'goal'))
            and len(components) == 17 and REQUIRED_COMPONENTS <= names
            and all(row.get('exit_code') == 0 for row in components.values())
            and raw.get('simulator_exit_code') == 0,
        'dds_alive_until_shutdown': shutdown.get('alive_before_signal') is True,
        'ros_communication_continuous': shutdown.get('ros_communication_healthy') is True
            and raw.get('clock_regressions') == 0 and len(raw.get('initializations', [])) == 1
            and set(raw.get('graph_contract', {})) == {'final_only_collision_monitor',
                'obstacle_truth_isolated', 'robot_truth_isolated'}
            and all(v is True for v in raw.get('graph_contract', {}).values())
            and bool(raw.get('parameter_checks'))
            and all(v is True for v in raw.get('parameter_checks', {}).values()),
        'expected_shutdown_after_navigation_and_stop': shutdown.get('navigation_and_stop_complete') is True
            and shutdown.get('signal') == 'SIGINT'
            and [row for row in signals if row[0] == 'dds'] ==
                [['dds', shutdown.get('pid'), 'SIGINT']],
        'dds_log_saved': proof.get('dds_log_saved') is True and bool(proof.get('dds_log_sha256')),
        'exact_nonzero_dds_code_saved': type(code) is int and code != 0
            and code not in (-9, -15) and proof.get('dds_returncode') == code,
        'no_signal_escalation': bool(signals) and all(row[-1] == 'SIGINT' for row in signals),
        'dds_pid_gone': proof.get('dds_pid_gone') is True,
        'dds_port_released': proof.get('dds_port_released') is True,
        'no_project_residuals': raw.get('remaining_owned_pids') == []
            and raw.get('remaining_project_processes') == []
            and proof.get('remaining_owned_pids') == []
            and proof.get('remaining_project_processes') == [],
        'host_graphics_checked_without_fault': proof.get('host_graphics_checked_without_fault') is True
            and 'gpu_monitor_error' in raw and raw['gpu_monitor_error'] is None,
        'no_other_error': not any(raw.get(k) for k in ('error', 'cleanup_error', 'metric_error', 'graph_error')),
        'runner_finished_normally': proof.get('runner_exit_code') in (1, 4)
            and raw.get('status') in ('FAILED', 'PENDING_PORT_CHECK') and not raw.get('startup_check_completed'),
    }
    eligible = all(checks.values())
    return dict(status='SUCCESS_WITH_WARNING' if eligible else 'FAILED',
                exception='DDS_TEARDOWN_EXCEPTION' if eligible else None,
                dds_returncode=code, checks=checks,
                failed_checks=[k for k, v in checks.items() if not v])

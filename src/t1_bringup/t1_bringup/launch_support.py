"""Shared W03 launch arguments, validation and one-shot diagnostic action."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MODES = ('sim_only', 'mapping', 'navigation', 'evaluation')


def launch_arguments():
    return [
        DeclareLaunchArgument('mode', default_value='sim_only', choices=MODES),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('upstream_path', default_value=EnvironmentVariable('T1_UPSTREAM', default_value='')),
        DeclareLaunchArgument('assets_path', default_value=EnvironmentVariable('T1_ASSETS', default_value='')),
        DeclareLaunchArgument('task', default_value='t1_walk', choices=['t1_walk']),
        DeclareLaunchArgument('params_file', default_value=str(
            Path(get_package_share_directory('t1_mujoco')) / 'config/runtime.yaml')),
    ]


def validate_request(mode, enable_sim):
    if mode not in MODES:
        raise RuntimeError(f'Unknown mode {mode!r}; expected one of {MODES}')
    if enable_sim not in ('true', 'false'):
        raise RuntimeError('enable_sim must be true or false')
    if enable_sim == 'true' and mode != 'sim_only':
        raise RuntimeError('W04 simulation supports sim_only; future modes are not implemented')


def diagnostic_actions(context):
    params_file = Path(LaunchConfiguration('params_file').perform(context))
    if not params_file.is_file():
        raise RuntimeError(f'params_file does not exist: {params_file}')
    node = Node(
        package='t1_mujoco', executable='t1_diagnostic', name='t1_diagnostic', output='screen',
        parameters=[str(params_file), {
            name: ParameterValue(LaunchConfiguration(name), value_type=str)
            for name in ('upstream_path', 'assets_path', 'mode', 'task')
        }, {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    )

    def finished(event, _context):
        # launch otherwise reports success even when a child exited nonzero.
        if event.returncode != 0:
            raise RuntimeError(f't1_diagnostic exited with code {event.returncode}')
        return [EmitEvent(event=Shutdown(reason='W03 diagnostic completed; no physics started.'))]

    return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=finished)), node]


def simulation_arguments():
    return [DeclareLaunchArgument(name, default_value=default) for name, default in (
        ('headless', 'true'), ('target_rtf', '1.0'), ('enable_policy_cmd_input', 'false'),
        ('max_sim_time', '0.0'), ('scene', ''), ('run_id', ''),
        # W05 calibrated v_des->u_pol contract; '' keeps /cmd_vel UNCALIBRATED.
        ('command_map_file', str(Path(get_package_share_directory('t1_mujoco')) / 'config/command_map.yaml')),
    )]


def simulation_actions(context):
    validate_request(LaunchConfiguration('mode').perform(context), 'true')
    params_file = Path(LaunchConfiguration('params_file').perform(context))
    if not params_file.is_file():
        raise RuntimeError(f'params_file does not exist: {params_file}')
    values = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
              for name, kind in (
                  ('upstream_path', str), ('assets_path', str), ('mode', str), ('task', str),
                  ('headless', bool), ('target_rtf', float), ('enable_policy_cmd_input', bool),
                  ('max_sim_time', float), ('scene', str), ('run_id', str), ('command_map_file', str))}
    values['use_sim_time'] = False
    node = Node(package='t1_mujoco', executable='t1_sim', name='t1_sim', output='screen',
                parameters=[str(params_file), values])

    def finished(event, _context):
        if event.returncode != 0:
            raise RuntimeError(f't1_sim exited with code {event.returncode}')
        return [EmitEvent(event=Shutdown(reason='t1_sim completed'))]
    return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=finished)), node]

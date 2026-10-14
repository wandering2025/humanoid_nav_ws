"""Shared W03 launch arguments, validation and one-shot diagnostic action."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MODES = ('sim_only', 'mapping', 'localization', 'navigation', 'evaluation')


def launch_arguments(default_mode='sim_only'):
    return [
        DeclareLaunchArgument('mode', default_value=default_mode, choices=MODES),
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
    if enable_sim == 'true' and mode in ('navigation', 'evaluation'):
        raise RuntimeError('navigation/evaluation require W10/W11; use mapping or localization for W09')


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


def simulation_arguments(default_scene='', map_rviz=False):
    return [DeclareLaunchArgument(name, default_value=default) for name, default in (
        ('headless', 'true'), ('target_rtf', '1.0'), ('enable_policy_cmd_input', 'false'),
        ('max_sim_time', '0.0'), ('scene', default_scene), ('run_id', ''),
        ('rviz', 'false'),
        ('rviz_config', str(Path(get_package_share_directory('t1_navigation' if map_rviz else 't1_description')) / ('config/mapping.rviz' if map_rviz else 'config/scene.rviz'))),
        # W05 calibrated v_des->u_pol contract; '' keeps /cmd_vel UNCALIBRATED.
        ('command_map_file', str(Path(get_package_share_directory('t1_mujoco')) / 'config/command_map.yaml')),
    )]


def simulation_actions(context):
    validate_request(LaunchConfiguration('mode').perform(context), 'true')
    if LaunchConfiguration('use_sim_time').perform(context) != 'true':
        raise RuntimeError('simulation consumers require use_sim_time=true')
    params_file = Path(LaunchConfiguration('params_file').perform(context))
    if not params_file.is_file():
        raise RuntimeError(f'params_file does not exist: {params_file}')
    values = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
              for name, kind in (
                  ('upstream_path', str), ('assets_path', str), ('mode', str), ('task', str),
                  ('headless', bool), ('target_rtf', float), ('enable_policy_cmd_input', bool),
                  ('max_sim_time', float), ('scene', str), ('run_id', str), ('command_map_file', str))}
    if LaunchConfiguration('mode').perform(context) != 'sim_only':
        if LaunchConfiguration('enable_policy_cmd_input').perform(context) != 'false':
            raise RuntimeError('mapping/localization forbid policy debug commands')
        if not LaunchConfiguration('command_map_file').perform(context):
            raise RuntimeError('mapping/localization require calibrated command_map_file')
    values['use_sim_time'] = False
    node = Node(package='t1_mujoco', executable='t1_sim', name='t1_sim', output='screen',
                parameters=[str(params_file), values])
    assets = LaunchConfiguration('assets_path').perform(context)
    if not assets:
        # System launch Python need not import venv's booster_assets package.
        # The explicit environment and default binding share the same baseline.
        import os
        upstream = Path(LaunchConfiguration('upstream_path').perform(context))
        assets = os.environ.get('T1_ASSETS', str(upstream.parent / 'booster_assets'))
    from .description import robot_description
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               name='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description(assets),
                            'use_sim_time': True, 'publish_frequency': 100.0}])
    consumers = [rsp]
    if LaunchConfiguration('rviz').perform(context) == 'true':
        rviz_config = Path(LaunchConfiguration('rviz_config').perform(context))
        if not rviz_config.is_file():
            raise RuntimeError(f'rviz_config does not exist: {rviz_config}')
        consumers.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                              arguments=['-d', str(rviz_config)], output='screen',
                              parameters=[{'use_sim_time': True}]))

    def finished(event, _context):
        if event.returncode != 0:
            raise RuntimeError(f't1_sim exited with code {event.returncode}')
        return [EmitEvent(event=Shutdown(reason='t1_sim completed'))]
    return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=finished)), node, *consumers]

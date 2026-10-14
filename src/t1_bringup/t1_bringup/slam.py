"""Exclusive W09 consumers, using installed Jazzy lifecycle launch semantics."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def arguments():
    share = Path(get_package_share_directory('t1_navigation'))
    return [DeclareLaunchArgument(name, default_value=str(share / path)) for name, path in (
        ('slam_params_file', 'config/slam.yaml'),
        ('localization_params_file', 'config/localization.yaml'),
        ('map', 'maps/static_world.yaml'))]


def actions(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode == 'mapping':
        params = Path(LaunchConfiguration('slam_params_file').perform(context))
        if not params.is_file():
            raise RuntimeError(f'slam_params_file does not exist: {params}')
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('slam_toolbox')) / 'launch/online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': str(params),
                              'autostart': 'true', 'use_lifecycle_manager': 'false'}.items())]
    if mode == 'localization':
        params = Path(LaunchConfiguration('localization_params_file').perform(context))
        map_file = Path(LaunchConfiguration('map').perform(context))
        if not params.is_file() or not map_file.is_file():
            raise RuntimeError(f'localization requires existing params and map: {params}, {map_file}')
        # Manager configures/activates map_server first, then AMCL. No SLAM.
        return [
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[str(params), {'use_sim_time': True, 'yaml_filename': str(map_file)}], output='screen'),
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 parameters=[str(params), {'use_sim_time': True}], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': ['map_server', 'amcl'], 'bond_timeout': 4.0}]),
        ]
    return []


def full_actions(context):
    from .launch_support import simulation_actions
    return simulation_actions(context) + actions(context)

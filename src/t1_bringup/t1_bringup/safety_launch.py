from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def arguments():
    return [DeclareLaunchArgument('command_source', default_value='teleop', choices=['teleop','navigation']),
            DeclareLaunchArgument('safety_params_file', default_value=str(
                Path(get_package_share_directory('t1_navigation'))/'config/safety.yaml'))]


def actions(context):
    if LaunchConfiguration('enable_policy_cmd_input').perform(context) == 'true':
        return []  # explicitly isolated sim_only calibration/debug path
    path = LaunchConfiguration('safety_params_file').perform(context)
    source = LaunchConfiguration('command_source').perform(context)
    if LaunchConfiguration('mode').perform(context) == 'mapping' and source != 'teleop':
        raise RuntimeError('mapping requires teleop source')
    if not Path(path).is_file():
        raise RuntimeError('missing safety_params_file')
    return [Node(package=package, executable=exe, name=name, output='screen',
                 parameters=[path, extra], remappings=remaps)
            for package,exe,name,extra,remaps in (
                ('t1_bringup','command_gate','t1_command_gate',{'command_source':source},[]),
                ('nav2_velocity_smoother','velocity_smoother','velocity_smoother',{},
                 [('cmd_vel','/cmd_vel_nav'),('cmd_vel_smoothed','/cmd_vel_smoothed')]),
                ('nav2_collision_monitor','collision_monitor','collision_monitor',{},[]),
                ('nav2_lifecycle_manager','lifecycle_manager','lifecycle_manager_safety',
                 {'use_sim_time':True,'autostart':True,'node_names':['velocity_smoother','collision_monitor'],
                  'bond_timeout':1.0,'attempt_respawn_reconnection':False},[]))]

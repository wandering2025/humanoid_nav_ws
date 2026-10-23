from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from t1_bringup.launch_support import launch_arguments, simulation_arguments
from t1_bringup.navigation_launch import arguments as navigation_arguments, full_actions
from t1_bringup.slam import arguments


def generate_launch_description():
    rviz = str(Path(get_package_share_directory('t1_navigation'))/'config/navigation.rviz')
    return LaunchDescription(launch_arguments('navigation') +
                             simulation_arguments('static_world', rviz_config=rviz) +
                             arguments('navigation') + navigation_arguments() +
                             [OpaqueFunction(function=full_actions)])

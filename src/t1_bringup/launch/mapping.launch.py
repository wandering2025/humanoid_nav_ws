from launch import LaunchDescription
from launch.actions import OpaqueFunction
from t1_bringup.launch_support import launch_arguments, simulation_arguments
from t1_bringup.slam import arguments, full_actions


def generate_launch_description():
    return LaunchDescription(launch_arguments('mapping') +
                             simulation_arguments('static_world', map_rviz=True) + arguments() +
                             [OpaqueFunction(function=full_actions)])

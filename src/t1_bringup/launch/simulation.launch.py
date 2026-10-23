from t1_bringup.slam import arguments
from t1_bringup.navigation_launch import arguments as navigation_arguments, full_actions
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from t1_bringup.launch_support import launch_arguments, simulation_arguments, simulation_actions


def generate_launch_description():
    return LaunchDescription(launch_arguments() + simulation_arguments() + arguments() + navigation_arguments() + [
        OpaqueFunction(function=full_actions),
    ])

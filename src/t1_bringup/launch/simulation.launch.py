from launch import LaunchDescription
from launch.actions import OpaqueFunction
from t1_bringup.launch_support import launch_arguments, simulation_arguments, simulation_actions


def generate_launch_description():
    return LaunchDescription(launch_arguments() + simulation_arguments() + [
        OpaqueFunction(function=simulation_actions),
    ])

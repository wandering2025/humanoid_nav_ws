from launch import LaunchDescription
from launch.actions import OpaqueFunction
from t1_bringup.launch_support import diagnostic_actions, launch_arguments


def generate_launch_description():
    return LaunchDescription(launch_arguments() + [OpaqueFunction(function=diagnostic_actions)])

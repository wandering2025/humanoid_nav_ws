from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from t1_bringup.launch_support import diagnostic_actions, launch_arguments, validate_request


def configure(context):
    validate_request(LaunchConfiguration('mode').perform(context), LaunchConfiguration('enable_sim').perform(context))
    # TODO W04: replace the explicit guard with t1_sim, using use_sim_time=false.
    # TODO W06/W09/W10/W11: add consumers with use_sim_time from launch.
    return diagnostic_actions(context)


def generate_launch_description():
    return LaunchDescription(launch_arguments() + [
        DeclareLaunchArgument('enable_sim', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=configure),
    ])

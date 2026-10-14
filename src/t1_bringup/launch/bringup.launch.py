from t1_bringup.slam import arguments
from t1_bringup.navigation_launch import arguments as navigation_arguments, full_actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from t1_bringup.launch_support import diagnostic_actions, launch_arguments, validate_request, simulation_actions, simulation_arguments


def configure(context):
    validate_request(LaunchConfiguration('mode').perform(context), LaunchConfiguration('enable_sim').perform(context))
    if LaunchConfiguration('enable_sim').perform(context) == 'true':
        return full_actions(context)
    # TODO W06/W09/W10/W11: add consumers with use_sim_time from launch.
    return diagnostic_actions(context)


def generate_launch_description():
    return LaunchDescription(launch_arguments() + simulation_arguments() + arguments() + navigation_arguments() + [
        DeclareLaunchArgument('enable_sim', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=configure),
    ])

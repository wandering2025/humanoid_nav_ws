"""Opt-in live SLAM + unchanged Nav2 and safety, no saved map or AMCL."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from t1_bringup.launch_support import launch_arguments, simulation_arguments
from t1_bringup.slam import arguments
from t1_bringup.navigation_launch import arguments as nav_arguments, full_actions

def generate_launch_description():
    return LaunchDescription(launch_arguments('advanced_mapping') +
        simulation_arguments('advanced_world') + arguments('navigation') +
        nav_arguments() + [OpaqueFunction(function=full_actions)])

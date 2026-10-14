"""Test-only transport interruption, preserving real sensor generation/physics."""
from launch import LaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch_ros.actions import SetRemap
from t1_bringup.launch_support import launch_arguments, simulation_arguments, simulation_actions
from t1_bringup.slam import arguments
from t1_bringup.safety_launch import actions


def configure(context):
    return [GroupAction([SetRemap(src='/'+t,dst='/w10/raw_'+t) for t in ('scan','odom','tf')]
                        + simulation_actions(context))] + actions(context)


def generate_launch_description():
    return LaunchDescription(launch_arguments()+simulation_arguments()+arguments()+[
        OpaqueFunction(function=configure)])

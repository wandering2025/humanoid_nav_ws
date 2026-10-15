"""W11 Nav2 servers, single command adapter and readiness-gated lifecycle startup."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RAW_CMD = '/t1/nav/cmd_vel_raw'
NAV2_NODES = (('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
              ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'))


def arguments():
    share = Path(get_package_share_directory('t1_navigation'))
    return [
        DeclareLaunchArgument('navigation_params_file', default_value=str(share/'config/navigation.yaml')),
        DeclareLaunchArgument('nav_to_pose_bt_xml', default_value=str(share/'behavior_trees/navigate_to_pose_w11.xml')),
        # Optional one-shot approximate AMCL initialization 'x y yaw' (map frame);
        # '' leaves it to RViz 2D Pose Estimate or `ros2 run t1_bringup initial_pose`.
        DeclareLaunchArgument('initial_pose', default_value=''),
        # Wall seconds before creating the Nav2 server group. Observed on this host:
        # with ~17 processes discovering at once, a lifecycle change_state response
        # was dropped and the (timeout-less) Jazzy lifecycle manager hung. Activation
        # itself is still gated by navigation_starter readiness checks.
        DeclareLaunchArgument('nav_start_delay', default_value='8.0'),
        # Localization/safety lifecycle managers start this long after their nodes, so
        # the managed services finish discovery before the first change_state. Suite v1
        # S04 hung on a lost map_server configure response (W11 C3, ADR-016).
        DeclareLaunchArgument('lifecycle_start_delay', default_value='3.0'),
    ]


def actions(context):
    if LaunchConfiguration('mode').perform(context) not in ('navigation', 'advanced_mapping'):
        return []
    params = Path(LaunchConfiguration('navigation_params_file').perform(context))
    tree = Path(LaunchConfiguration('nav_to_pose_bt_xml').perform(context))
    command_map = LaunchConfiguration('command_map_file').perform(context)
    for path in (params, tree, Path(command_map)):
        if not path.is_file():
            raise RuntimeError(f'navigation requires existing file: {path}')
    nodes = []
    for package, name in NAV2_NODES:
        extra = {'use_sim_time': True}
        if name == 'bt_navigator':
            extra['default_nav_to_pose_bt_xml'] = str(tree)
        remaps = [('cmd_vel', RAW_CMD)] if name in ('controller_server', 'behavior_server') else []
        nodes.append(Node(package=package, executable=name, name=name, output='screen',
                          parameters=[str(params), extra], remappings=remaps))
    nodes += [
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[str(params)]),
        Node(package='t1_bringup', executable='navigation_adapter', name='t1_navigation_adapter',
             output='screen', parameters=[str(params), {'command_map_file': command_map}]),
        Node(package='t1_bringup', executable='navigation_starter', name='t1_navigation_starter',
             output='screen', parameters=[str(params)]),
    ]
    delay = float(LaunchConfiguration('nav_start_delay').perform(context))
    if not 0 <= delay <= 60:
        raise RuntimeError('nav_start_delay must be within [0, 60] wall seconds')
    pose = LaunchConfiguration('initial_pose').perform(context).split()
    if pose:
        if len(pose) != 3:
            raise RuntimeError("initial_pose must be 'x y yaw' or empty")
        # Also delayed: polling /amcl/get_state during the initial discovery burst made
        # AMCL's response time out and blocked its lifecycle bringup (W11 C2).
        nodes.append(ExecuteProcess(cmd=['ros2', 'run', 't1_bringup', 'initial_pose', *pose],
                                    output='screen', name='t1_initial_pose_once'))
    from .launch_support import rviz_nodes
    return [TimerAction(period=delay, actions=nodes + rviz_nodes(context))]


def full_actions(context):
    from .slam import full_actions as localization_actions
    return localization_actions(context) + actions(context)

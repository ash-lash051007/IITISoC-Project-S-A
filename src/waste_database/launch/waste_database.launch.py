import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- toggle for task managers: OFF by default. While you're still in
    # the "detect everything first" phase, targets should just sit as
    # PENDING -- task managers auto-claim any PENDING target the moment it
    # appears, which is exactly what you don't want yet. Turn this on with:
    #   ros2 launch waste_database waste_database.launch.py enable_task_managers:=true
    # once you're ready to test actual task execution/allocation. ---
    enable_task_managers_arg = DeclareLaunchArgument(
        'enable_task_managers',
        default_value='false',
        description='Set true to let robots auto-request and execute tasks.',
    )
    enable_task_managers = LaunchConfiguration('enable_task_managers')

    # --- the database node itself ---
    waste_database_node = Node(
        package='waste_database',
        executable='waste_database_node',
        name='waste_database_node',
        output='screen',
        parameters=[{'publish_rate_hz': 2.0}],
    )

    # --- one robot_state_bridge_node per robot, translating each robot's
    # real Nav2 odometry into the /robot_state topic the database expects ---
    robot_bridges_launch_path = os.path.join(
        get_package_share_directory('waste_database'),
        'launch',
        'robot_state_bridges.launch.py',
    )
    robot_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_bridges_launch_path)
    )

    # --- one task_manager_node per robot: requests tasks, drives to them
    # via Nav2, reports outcomes. Only launched if enable_task_managers is
    # true (see toggle above). Delayed slightly so the database node and
    # Nav2 action servers have a moment to come up first. ---
    task_managers_launch_path = os.path.join(
        get_package_share_directory('waste_database'),
        'launch',
        'task_managers.launch.py',
    )
    task_managers = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(task_managers_launch_path)
        )],
        condition=IfCondition(enable_task_managers),
    )

    # --- rosbridge: exposes ROS2 topics over a websocket so the browser
    # dashboard can subscribe to them. Requires:
    #   sudo apt install ros-humble-rosbridge-server
    rosbridge_launch_path = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    )
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_path)
    )

    # --- auto-open the dashboard in the default browser, a few seconds
    # after launch so rosbridge has time to bind to port 9090 first.
    # Comment this block out (and remove it from the LaunchDescription
    # below) if you don't want a new browser tab every single launch,
    # e.g. during rapid dev/test cycles.
    dashboard_path = os.path.join(
        get_package_share_directory('waste_database'),
        'dashboard',
        'waste_dashboard.html',
    )
    open_dashboard = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['xdg-open', dashboard_path],
            output='screen',
        )],
    )

    return LaunchDescription([
        enable_task_managers_arg,
        waste_database_node,
        robot_bridges,
        task_managers,
        rosbridge,
        open_dashboard,
    ])

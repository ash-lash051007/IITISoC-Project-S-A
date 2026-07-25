import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
        waste_database_node,
        robot_bridges,
        rosbridge,
        open_dashboard,
    ])

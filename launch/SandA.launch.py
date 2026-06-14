import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    world_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'worlds', 'warehouse_world.world'
    )

    tb3_model_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', 'turtlebot3_waffle', 'model.sdf'
    )

    tb3_urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf', 'turtlebot3_waffle.urdf'
    )
    with open(tb3_urdf, 'r') as f:
        robot_desc = f.read()

    # ── Gazebo ──
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false'
        }.items()
    )

    # ── Robot 1 ──
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', tb3_model_sdf,
            '-x', '-2.0', '-y', '-2.0', '-z', '0.01',
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    )

    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': 'robot1/'
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    # ── Robot 2 (delayed 3 sec) ──
    spawn_robot2 = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'robot2',
                    '-file', tb3_model_sdf,
                    '-x', '2.0', '-y', '-2.0', '-z', '0.01',
                    '-robot_namespace', 'robot2'
                ],
                output='screen'
            )
        ]
    )

    robot2_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot2',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True,
            'frame_prefix': 'robot2/'
        }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot1,
        robot1_state_pub,
        spawn_robot2,
        robot2_state_pub,
    ])

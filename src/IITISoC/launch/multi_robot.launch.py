import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    world_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'worlds', 'warehouse_world.world'
    )

    tb3_model_sdf_robot1 = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'models', 'turtlebot3_waffle_robot1', 'model.sdf'
    )
    tb3_model_sdf_robot2 = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'models', 'turtlebot3_waffle_robot2', 'model.sdf'
    )
    tb3_model_sdf_robot3 = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'models', 'turtlebot3_waffle_robot3', 'model.sdf'
    )

    tb3_urdf = os.path.join(
        get_package_share_directory('iitisoc'),
        'urdf', 'turtlebot3_waffle_fixed.urdf'
    )

    # Process URDF through xacro for each robot, passing namespace as argument.
    # This bakes in correctly prefixed frame names (e.g. robot1/base_link)
    # directly into the robot description, so frame_prefix is NOT needed.
    # NOTE: namespace is passed WITH trailing slash (e.g. 'robot1/') because
    # the URDF uses ${namespace}base_link with no separator between them.
    robot1_desc = xacro.process_file(
        tb3_urdf,
        mappings={'namespace': 'robot1/'}
    ).toxml()

    robot2_desc = xacro.process_file(
        tb3_urdf,
        mappings={'namespace': 'robot2/'}
    ).toxml()
    
    robot3_desc = xacro.process_file(
        tb3_urdf,
        mappings={'namespace': 'robot3/'}
    ).toxml()


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
            'verbose': 'false',
            'gui': 'false'
        }.items()
    )

    # ── Robot 1 ──
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', tb3_model_sdf_robot1,
            '-x','20.0','-y','24.0','-z','0.05',
            '-robot_namespace', 'robot1'
        ],
        output='screen'
    )

    robot1_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot1',
        parameters=[{
            'robot_description': robot1_desc,
            'use_sim_time': True,
            # frame_prefix is intentionally absent — xacro namespace substitution
            # already produces prefixed frames. Adding frame_prefix here would
            # double-prefix everything to robot1/robot1/base_link.
        }],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('/robot_description', 'robot_description'),
        ],
        output='screen'
    )
    
    # ── Robot 2 (delayed 5 sec to avoid spawn collision) ──
    spawn_robot2 = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'robot2',
                    '-file', tb3_model_sdf_robot2,
                    '-x', '26.0', '-y', '5', '-z', '0.05',
                    '-robot_namespace', 'robot2'
                ],
                output='screen'
            )
        ]
    )

    robot2_state_pub = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='robot2',
                parameters=[{
                    'robot_description': robot2_desc,
                    'use_sim_time': True,
                    # Same as robot1 — frame_prefix not needed, xacro handles it.
                }],
                remappings=[
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),
                    ('/robot_description', 'robot_description'),
                ],
                output='screen'
            )
        ]
    )
    
    # ── Robot 3 (delayed 10 sec to avoid spawn collision) ──
    spawn_robot3 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'robot3',
                    '-file', tb3_model_sdf_robot3,
                    '-x', '32.0', '-y', '25.0', '-z', '0.05',
                    '-robot_namespace', 'robot3'
                ],
                output='screen'
            )
        ]
    )

    robot3_state_pub = TimerAction(
        period=10.5,
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='robot3',
                parameters=[{
                    'robot_description': robot3_desc,
                    'use_sim_time': True,
                }],
                remappings=[
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),
                    ('/robot_description', 'robot_description'),
                ],
                output='screen'
            )
        ]
    )

    

    return LaunchDescription([
        gazebo,
        spawn_robot1,
        robot1_state_pub,
        spawn_robot2,
        robot2_state_pub,
        spawn_robot3,
        robot3_state_pub,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_allocator',
            executable='allocator_node',
            name='task_allocator_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'strategy': 'hungarian'}
            ],
            remappings=[
                ('/waste_targets', '/waste_database/targets'),
                ('/robot_states', '/waste_database/robot_states'),
                ('/task_assignments', '/waste_database/task_assignments'),
            ]
        ),
Node(
            package='task_allocator',
            executable='goal_sender_node',
            name='goal_sender_robot1',
            namespace='robot1',
            output='screen',
            parameters=[
                {'robot_name': 'robot1'},
                {'use_sim_time': True}
            ]
        ),

        # Goal Sender Node for Robot 2
        Node(
            package='task_allocator',
            executable='goal_sender_node',
            name='goal_sender_robot2',
            namespace='robot2',
            output='screen',
            parameters=[
                {'robot_name': 'robot2'},
                {'use_sim_time': True}
            ]
        ),

        # Goal Sender Node for Robot 3
        Node(
            package='task_allocator',
            executable='goal_sender_node',
            name='goal_sender_robot3',
            namespace='robot3',
            output='screen',
            parameters=[
                {'robot_name': 'robot3'},
                {'use_sim_time': True}
            ]
        ),
    ])        

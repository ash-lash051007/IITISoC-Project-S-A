from launch import LaunchDescription
from launch_ros.actions import Node


ROBOTS = [
    {'name': 'robot1'},
    {'name': 'robot2'},
    {'name': 'robot3'},
]


def generate_launch_description():
    nodes = []
    for r in ROBOTS:
        nodes.append(Node(
            package='waste_database',
            executable='task_manager_node',
            name=f"task_manager_{r['name']}",
            output='screen',
            parameters=[{
                'robot_name': r['name'],
                'odom_topic': f"/{r['name']}/odom",
                'navigate_action': f"/{r['name']}/navigate_to_pose",
            }],
        ))
    return LaunchDescription(nodes)

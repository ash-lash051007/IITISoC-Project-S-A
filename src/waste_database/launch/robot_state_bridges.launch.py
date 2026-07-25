from launch import LaunchDescription
from launch_ros.actions import Node


ROBOTS = [
    {'name': 'robot1', 'id': 1},
    {'name': 'robot2', 'id': 2},
    {'name': 'robot3', 'id': 3},
]


def generate_launch_description():
    nodes = []
    for r in ROBOTS:
        nodes.append(Node(
            package='waste_database',
            executable='robot_state_bridge_node',
            name=f"robot_state_bridge_{r['name']}",
            output='screen',
            parameters=[{
                'robot_name': r['name'],
                'robot_id': r['id'],
                'odom_topic': f"/{r['name']}/odom",
            }],
        ))
    return LaunchDescription(nodes)

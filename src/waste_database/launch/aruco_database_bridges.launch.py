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
            executable='aruco_database_bridge_node',
            name=f"aruco_bridge_{r['name']}",
            output='screen',
            parameters=[{
                'robot_name': r['name'],
                'log_topic': f"/{r['name']}/aruco_detections/log",
            }],
        ))
    return LaunchDescription(nodes)

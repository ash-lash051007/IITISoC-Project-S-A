from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    aruco_robot1 = Node(
        package='iitisoc',
        executable='aruco_detector',
        name='aruco_detector_robot1',
        parameters=[{'robot_name': 'robot1'}],
        output='screen'
    )

    aruco_robot2 = Node(
        package='iitisoc',
        executable='aruco_detector',
        name='aruco_detector_robot2',
        parameters=[{'robot_name': 'robot2'}],
        output='screen'
    )
    
    aruco_robot3 = Node(
        package='iitisoc',
        executable='aruco_detector',
        name='aruco_detector_robot3',
        parameters=[{'robot_name': 'robot3'}],
        output='screen'
    )

    return LaunchDescription([
        aruco_robot1,
        aruco_robot2,
        aruco_robot3,
    ])

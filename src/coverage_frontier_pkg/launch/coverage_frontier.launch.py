import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    pkg_share = get_package_share_directory("coverage_frontier_pkg")
    default_rooms_yaml = os.path.join(pkg_share, "config", "rooms.yaml")

    # 1. Declare use_sim_time argument (defaults to true)
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    map_pgm_arg = DeclareLaunchArgument(
        "map_pgm_path", description="Path to warehouse_map.pgm"
    )
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml_path", description="Path to warehouse_map.yaml"
    )
    rooms_yaml_arg = DeclareLaunchArgument(
        "rooms_yaml_path", default_value=default_rooms_yaml
    )

    robot_ids = ["robot1", "robot2", "robot3"]
    home_rooms = {
        "robot1": "warehouse",
        "robot2": "storage",
        "robot3": "office",
    }

    grid_server = Node(
        package="coverage_frontier_pkg",
        executable="coverage_grid_server",
        name="coverage_grid_server",
        output="screen",
        parameters=[
            {
                "map_pgm_path": LaunchConfiguration("map_pgm_path"),
                "map_yaml_path": LaunchConfiguration("map_yaml_path"),
                "rooms_yaml_path": LaunchConfiguration("rooms_yaml_path"),
                "robot_ids": robot_ids,
            }
        ],
    )

    door_server = Node(
        package="coverage_frontier_pkg",
        executable="door_reservation_server",
        name="door_reservation_server",
        output="screen",
    )

    coordinator = Node(
        package="coverage_frontier_pkg",
        executable="coordinator_node",
        name="coordinator_node",
        output="screen",
        parameters=[{"robot_ids": robot_ids}],
    )

    robot_nodes = [
        Node(
            package="coverage_frontier_pkg",
            executable="robot_coverage_node",
            name=f"{rid}_coverage_node",
            output="screen",
            parameters=[
                {
                    "robot_id": rid,
                    "home_room": home_rooms[rid],
                    "rooms_yaml_path": LaunchConfiguration("rooms_yaml_path"),
                    "spin_interval_m": 2.5,
                }
            ],
        )
        for rid in robot_ids
    ]

    return LaunchDescription(
        [
            use_sim_time_arg,
            # 2. Automatically apply use_sim_time to ALL nodes in this launch file
            SetParameter(
                name="use_sim_time", value=LaunchConfiguration("use_sim_time")
            ),
            map_pgm_arg,
            map_yaml_arg,
            rooms_yaml_arg,
            grid_server,
            door_server,
            coordinator,
        ]
        + robot_nodes
    )

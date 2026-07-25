import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():

    nav2_params_r1 = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'config', 'nav2_params_robot1.yaml'
    )

    nav2_params_r2 = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'config', 'nav2_params_robot2.yaml'
    )

    nav2_params_r3 = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'config', 'nav2_params_robot3.yaml'
    )

    map_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws', 'src', 'IITISoC',
        'config', 'warehouse_map.yaml'
    )

    # ── ROBOT 1 ──

    r1_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='robot1',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
    )

    r1_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot1',
        output='screen',
        parameters=[nav2_params_r1, {'use_sim_time': True}],
        remappings=[('scan', '/robot1/scan'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static')]
    )

    r1_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='robot1',
        output='screen',
        parameters=[
            nav2_params_r1,
            {'use_sim_time': True,
             'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
             'FollowPath.desired_linear_vel': 0.26,
             'FollowPath.max_linear_accel': 2.5,
             'FollowPath.max_linear_decel': 2.5,
             'FollowPath.lookahead_dist': 0.6,
             'FollowPath.min_lookahead_dist': 0.3,
             'FollowPath.max_lookahead_dist': 0.9,
             'FollowPath.lookahead_time': 1.5,
             'FollowPath.rotate_to_heading_angular_vel': 1.0,
             'FollowPath.transform_tolerance': 0.2,
             'FollowPath.use_velocity_scaled_lookahead_dist': False,
             'FollowPath.min_approach_linear_velocity': 0.05,
             'FollowPath.approach_velocity_scaling_dist': 0.6,
             'FollowPath.use_collision_detection': True,
             'FollowPath.max_allowed_time_to_collision_up_to_carrot': 1.0,
             'FollowPath.use_regulated_linear_velocity_scaling': True,
             'FollowPath.use_fixed_curvature_lookahead': False,
             'FollowPath.curvature_feedforward_gain': 1.0,
             'FollowPath.use_cost_regulated_linear_velocity_scaling': False,
             'FollowPath.regulated_linear_scaling_min_radius': 0.9,
             'FollowPath.regulated_linear_scaling_min_speed': 0.25,
             'FollowPath.use_rotate_to_heading': True,
             'FollowPath.rotate_to_heading_min_angle': 0.785,
             'FollowPath.max_angular_accel': 3.2,
             'FollowPath.max_robot_pose_search_dist': 10.0,
             'local_costmap.robot_base_frame': 'robot1/base_footprint',
             'local_costmap.global_frame': 'robot1/odom',
             'local_costmap.plugins': ['voxel_layer', 'inflation_layer'],
             'local_costmap.voxel_layer.plugin': 'nav2_costmap_2d::VoxelLayer',
             'local_costmap.voxel_layer.observation_sources': 'scan',
             'local_costmap.voxel_layer.scan.topic': '/robot1/scan',
             'local_costmap.voxel_layer.scan.data_type': 'LaserScan',
             'local_costmap.voxel_layer.scan.clearing': True,
             'local_costmap.voxel_layer.scan.marking': True,
             'local_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
             'local_costmap.inflation_layer.cost_scaling_factor': 3.0,
             'local_costmap.inflation_layer.inflation_radius': 0.55,
             'local_costmap.transform_tolerance': 2.0}
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),]
    )

    r1_smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='robot1',
        output='screen',
        parameters=[nav2_params_r1, {'use_sim_time': True}],
    )

    r1_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='robot1',
        output='screen',
        parameters=[
            nav2_params_r1,
            {'use_sim_time': True,
             'global_costmap.robot_base_frame': 'robot1/base_footprint',
             'global_costmap.global_frame': 'map',
             'global_costmap.plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
             'global_costmap.obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
             'global_costmap.obstacle_layer.observation_sources': 'scan',
             'global_costmap.obstacle_layer.scan.topic': '/robot1/scan',
             'global_costmap.obstacle_layer.scan.data_type': 'LaserScan',
             'global_costmap.obstacle_layer.scan.clearing': True,
             'global_costmap.obstacle_layer.scan.marking': True,
             'global_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
             'global_costmap.inflation_layer.cost_scaling_factor': 3.0,
             'global_costmap.inflation_layer.inflation_radius': 0.55,
             'global_costmap.transform_tolerance': 2.0}
        ],
    )

    r1_behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='robot1',
        output='screen',
        parameters=[nav2_params_r1, {'use_sim_time': True}],
    )

    r1_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='robot1',
        output='screen',
        parameters=[nav2_params_r1, {'use_sim_time': True}],
    )

    r1_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='robot1',
        output='screen',
        parameters=[nav2_params_r1, {'use_sim_time': True}],
    )

    r1_velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='robot1',
        output='screen',
        parameters=[nav2_params_r1, {'use_sim_time': True}],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')]
    )

    r1_lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': ['map_server', 'amcl']
        }]
    )

    r1_lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace='robot1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # ── ROBOT 2 ──

    r2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='robot2',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
    )

    r2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot2',
        output='screen',
        parameters=[nav2_params_r2, {'use_sim_time': True}],
        remappings=[('scan', '/robot2/scan'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static')]
    )

    r2_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='robot2',
        output='screen',
        parameters=[
            nav2_params_r2,
            {'use_sim_time': True,
             'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
             'FollowPath.desired_linear_vel': 0.26,
             'FollowPath.max_linear_accel': 2.5,
             'FollowPath.max_linear_decel': 2.5,
             'FollowPath.lookahead_dist': 0.6,
             'FollowPath.min_lookahead_dist': 0.3,
             'FollowPath.max_lookahead_dist': 0.9,
             'FollowPath.lookahead_time': 1.5,
             'FollowPath.rotate_to_heading_angular_vel': 1.0,
             'FollowPath.transform_tolerance': 0.2,
             'FollowPath.use_velocity_scaled_lookahead_dist': False,
             'FollowPath.min_approach_linear_velocity': 0.05,
             'FollowPath.approach_velocity_scaling_dist': 0.6,
             'FollowPath.use_collision_detection': True,
             'FollowPath.max_allowed_time_to_collision_up_to_carrot': 1.0,
             'FollowPath.use_regulated_linear_velocity_scaling': True,
             'FollowPath.use_fixed_curvature_lookahead': False,
             'FollowPath.curvature_feedforward_gain': 1.0,
             'FollowPath.use_cost_regulated_linear_velocity_scaling': False,
             'FollowPath.regulated_linear_scaling_min_radius': 0.9,
             'FollowPath.regulated_linear_scaling_min_speed': 0.25,
             'FollowPath.use_rotate_to_heading': True,
             'FollowPath.rotate_to_heading_min_angle': 0.785,
             'FollowPath.max_angular_accel': 3.2,
             'FollowPath.max_robot_pose_search_dist': 10.0,
             'local_costmap.robot_base_frame': 'robot2/base_footprint',
             'local_costmap.global_frame': 'robot2/odom',
             'local_costmap.plugins': ['voxel_layer', 'inflation_layer'],
             'local_costmap.voxel_layer.plugin': 'nav2_costmap_2d::VoxelLayer',
             'local_costmap.voxel_layer.observation_sources': 'scan',
             'local_costmap.voxel_layer.scan.topic': '/robot2/scan',
             'local_costmap.voxel_layer.scan.data_type': 'LaserScan',
             'local_costmap.voxel_layer.scan.clearing': True,
             'local_costmap.voxel_layer.scan.marking': True,
             'local_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
             'local_costmap.inflation_layer.cost_scaling_factor': 3.0,
             'local_costmap.inflation_layer.inflation_radius': 0.55,
             'local_costmap.transform_tolerance': 2.0}
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),]
    )

    r2_smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='robot2',
        output='screen',
        parameters=[nav2_params_r2, {'use_sim_time': True}],
    )

    r2_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='robot2',
        output='screen',
        parameters=[
            nav2_params_r2,
            {'use_sim_time': True,
             'global_costmap.robot_base_frame': 'robot2/base_footprint',
             'global_costmap.global_frame': 'map',
             'global_costmap.plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
             'global_costmap.obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
             'global_costmap.obstacle_layer.observation_sources': 'scan',
             'global_costmap.obstacle_layer.scan.topic': '/robot2/scan',
             'global_costmap.obstacle_layer.scan.data_type': 'LaserScan',
             'global_costmap.obstacle_layer.scan.clearing': True,
             'global_costmap.obstacle_layer.scan.marking': True,
             'global_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
             'global_costmap.inflation_layer.cost_scaling_factor': 3.0,
             'global_costmap.inflation_layer.inflation_radius': 0.55,
             'global_costmap.transform_tolerance': 2.0}
        ],
    )

    r2_behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='robot2',
        output='screen',
        parameters=[nav2_params_r2, {'use_sim_time': True}],
    )

    r2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='robot2',
        output='screen',
        parameters=[nav2_params_r2, {'use_sim_time': True}],
    )

    r2_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='robot2',
        output='screen',
        parameters=[nav2_params_r2, {'use_sim_time': True}],
    )

    r2_velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='robot2',
        output='screen',
        parameters=[nav2_params_r2, {'use_sim_time': True}],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')]
    )

    r2_lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': ['map_server', 'amcl']
        }]
    )

    r2_lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace='robot2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # ── ROBOT 3 ──

    r3_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='robot3',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
    )

    r3_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='robot3',
        output='screen',
        parameters=[nav2_params_r3, {'use_sim_time': True}],
        remappings=[('scan', '/robot3/scan'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static')]
    )

    r3_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='robot3',
        output='screen',
        parameters=[
            nav2_params_r3,
            {'use_sim_time': True,
             'FollowPath.plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
             'FollowPath.desired_linear_vel': 0.26,
             'FollowPath.max_linear_accel': 2.5,
             'FollowPath.max_linear_decel': 2.5,
             'FollowPath.lookahead_dist': 0.6,
             'FollowPath.min_lookahead_dist': 0.3,
             'FollowPath.max_lookahead_dist': 0.9,
             'FollowPath.lookahead_time': 1.5,
             'FollowPath.rotate_to_heading_angular_vel': 1.0,
             'FollowPath.transform_tolerance': 0.2,
             'FollowPath.use_velocity_scaled_lookahead_dist': False,
             'FollowPath.min_approach_linear_velocity': 0.05,
             'FollowPath.approach_velocity_scaling_dist': 0.6,
             'FollowPath.use_collision_detection': True,
             'FollowPath.max_allowed_time_to_collision_up_to_carrot': 1.0,
             'FollowPath.use_regulated_linear_velocity_scaling': True,
             'FollowPath.use_fixed_curvature_lookahead': False,
             'FollowPath.curvature_feedforward_gain': 1.0,
             'FollowPath.use_cost_regulated_linear_velocity_scaling': False,
             'FollowPath.regulated_linear_scaling_min_radius': 0.9,
             'FollowPath.regulated_linear_scaling_min_speed': 0.25,
             'FollowPath.use_rotate_to_heading': True,
             'FollowPath.rotate_to_heading_min_angle': 0.785,
             'FollowPath.max_angular_accel': 3.2,
             'FollowPath.max_robot_pose_search_dist': 10.0,
             'local_costmap.robot_base_frame': 'robot3/base_footprint',
             'local_costmap.global_frame': 'robot3/odom',
             'local_costmap.plugins': ['voxel_layer', 'inflation_layer'],
             'local_costmap.voxel_layer.plugin': 'nav2_costmap_2d::VoxelLayer',
             'local_costmap.voxel_layer.observation_sources': 'scan',
             'local_costmap.voxel_layer.scan.topic': '/robot3/scan',
             'local_costmap.voxel_layer.scan.data_type': 'LaserScan',
             'local_costmap.voxel_layer.scan.clearing': True,
             'local_costmap.voxel_layer.scan.marking': True,
             'local_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
             'local_costmap.inflation_layer.cost_scaling_factor': 3.0,
             'local_costmap.inflation_layer.inflation_radius': 0.55,
             'local_costmap.transform_tolerance': 2.0}
        ],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('tf', '/tf'),
                    ('tf_static', '/tf_static'),]
    )

    r3_smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='robot3',
        output='screen',
        parameters=[nav2_params_r3, {'use_sim_time': True}],
    )

    r3_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='robot3',
        output='screen',
        parameters=[
            nav2_params_r3,
            {'use_sim_time': True,
             'global_costmap.robot_base_frame': 'robot3/base_footprint',
             'global_costmap.global_frame': 'map',
             'global_costmap.plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
             'global_costmap.obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
             'global_costmap.obstacle_layer.observation_sources': 'scan',
             'global_costmap.obstacle_layer.scan.topic': '/robot3/scan',
             'global_costmap.obstacle_layer.scan.data_type': 'LaserScan',
             'global_costmap.obstacle_layer.scan.clearing': True,
             'global_costmap.obstacle_layer.scan.marking': True,
             'global_costmap.inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
             'global_costmap.inflation_layer.cost_scaling_factor': 3.0,
             'global_costmap.inflation_layer.inflation_radius': 0.55,
             'global_costmap.transform_tolerance': 2.0}
        ],
    )

    r3_behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='robot3',
        output='screen',
        parameters=[nav2_params_r3, {'use_sim_time': True}],
    )

    r3_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='robot3',
        output='screen',
        parameters=[nav2_params_r3, {'use_sim_time': True}],
    )

    r3_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='robot3',
        output='screen',
        parameters=[nav2_params_r3, {'use_sim_time': True}],
    )

    r3_velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='robot3',
        output='screen',
        parameters=[nav2_params_r3, {'use_sim_time': True}],
        remappings=[('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')]
    )

    r3_lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='robot3',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': ['map_server', 'amcl']
        }]
    )

    r3_lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace='robot3',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 15.0,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    return LaunchDescription([
        # Robot 1 nodes
        r1_map_server,
        r1_amcl,
        r1_controller_server,
        r1_smoother_server,
        r1_planner_server,
        r1_behavior_server,
        r1_bt_navigator,
        r1_waypoint_follower,
        r1_velocity_smoother,

        # Robot 2 nodes
        r2_map_server,
        r2_amcl,
        r2_controller_server,
        r2_smoother_server,
        r2_planner_server,
        r2_behavior_server,
        r2_bt_navigator,
        r2_waypoint_follower,
        r2_velocity_smoother,

        # Robot 3 nodes
        r3_map_server,
        r3_amcl,
        r3_controller_server,
        r3_smoother_server,
        r3_planner_server,
        r3_behavior_server,
        r3_bt_navigator,
        r3_waypoint_follower,
        r3_velocity_smoother,

        # Robot 1 lifecycle managers
        TimerAction(period=15.0, actions=[r1_lifecycle_manager_localization]),
        TimerAction(period=22.0, actions=[r1_lifecycle_manager_navigation]),

        # Robot 2 lifecycle managers (slightly later)
        TimerAction(period=29.0, actions=[r2_lifecycle_manager_localization]),
        TimerAction(period=36.0, actions=[r2_lifecycle_manager_navigation]),
    
        # Robot 3 lifecycle managers (slightly later)
        TimerAction(period=43.0, actions=[r3_lifecycle_manager_localization]),
        TimerAction(period=50.0, actions=[r3_lifecycle_manager_navigation]),
    
    ])

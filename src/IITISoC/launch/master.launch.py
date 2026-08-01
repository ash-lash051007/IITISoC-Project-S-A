"""
master.launch.py
────────────────
Launches the entire IITISoC system in one command:
  1.  Gazebo + 3 robots          (multi_robot.launch.py)
  2.  Nav2 for all 3 robots      (nav2.launch.py)         → delayed 15 s
  3.  ArUco detection × 3        (aruco_detection.launch.py) → delayed 55 s
  4.  Waste database node        (waste_database.launch.py)  → delayed 58 s

Timing rationale
────────────────
  0  s  →  Gazebo starts
  5  s  →  robot1 spawned  (from multi_robot.launch.py)
 10  s  →  robot2 spawned
 15  s  →  robot3 spawned
 15  s  →  Nav2 nodes start (map_server, amcl, planners …)
 30  s  →  robot1 lifecycle managers activate  (Nav2 ready for r1)
 37  s  →  robot2 lifecycle managers activate
 44  s  →  robot3 lifecycle managers activate
 50  s  →  Nav2 fully active for all 3 robots
 55  s  →  ArUco detectors start (camera topics guaranteed up)
 58  s  →  Waste database starts (detection topics guaranteed up)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('iitisoc')
    launch_dir = os.path.join(pkg_share, 'launch')

    # ── 1. Gazebo + robot spawning ──────────────────────────────────────────
    # Robots spawn at t=5, t=10, t=15 s (as defined in multi_robot.launch.py)
    multi_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'multi_robot.launch.py')
        )
    )

    # ── 2. Nav2 for all 3 robots ─────────────────────────────────────────────
    # Start at t=15 s so robot3 is already spawned before Nav2 starts.
    # Nav2 internally uses TimerActions (from nav2.launch.py) to stagger
    # lifecycle managers:  r1 @ +15 s, r2 @ +22 s, r3 @ +29 s  relative
    # to when nav2.launch.py itself starts — so effective wall-clock times
    # are:  r1 active @ ~30 s, r2 @ ~37 s, r3 @ ~44 s from master start.
    nav2 = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[master] t=15s → Launching Nav2 for all robots'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'nav2.launch.py')
                )
            )
        ]
    )

    # ── 3. ArUco detection nodes (one per robot) ────────────────────────────
    # Start at t=55 s — well after all robots are spawned and cameras are up.
    # Camera topics (/robotN/camera/image_raw) are available ~5 s after spawn.
    aruco = TimerAction(
        period=55.0,
        actions=[
            LogInfo(msg='[master] t=55s → Launching ArUco detectors'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'aruco_detection.launch.py')
                )
            )
        ]
    )

    # ── 4. Waste database node ───────────────────────────────────────────────
    # Start at t=58 s — after detectors are up so it doesn't miss early msgs.
    waste_db = TimerAction(
        period=58.0,
        actions=[
            LogInfo(msg='[master] t=58s → Launching Waste Database node'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('waste_database'),
                        'launch',
                        'waste_database.launch.py'
                    )
                )
            )
        ]
    )

    return LaunchDescription([
        LogInfo(msg='[master] Starting IITISoC full system launch'),
        multi_robot,
        nav2,
        aruco,
        waste_db,
    ])

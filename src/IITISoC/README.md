# IITISoC — Multi Robot Waste Detection

## Project Overview
Autonomous waste detection system using 2 TurtleBot3
robots in a warehouse simulation built with
ROS2 Humble + Gazebo.

## Team
- Member 1 (Ash): Simulation + Navigation
- Member 2 (Shrawani): Perception + Coordination

## Features
- Custom warehouse Gazebo world
- 2 TurtleBot3 Waffle robots
- Nav2 autonomous navigation
- Collision avoidance between robots
- Systematic warehouse patrol
- ArUco marker detection
- Shared detection database
- Nearest-robot task allocation

## How to Run

### Full System (one command):
bash scripts/run_all.sh

### Manual Launch:
Terminal 1 - Simulation:
ros2 launch ~/ros2_ws/src/IITISoC/launch/SandA.launch.py

Terminal 2 - Detection:
python3 scripts/aruco_detector.py

Terminal 3 - Robot 1 Patrol:
python3 scripts/patrol_robot1.py

Terminal 4 - Robot 2 Patrol:
python3 scripts/patrol_robot2.py

## World Details
- Size: 15m x 15m warehouse
- 6 shelf units creating corridors
- 6 ArUco markers (IDs 0-5)
- 1 divider wall
- Loading area with boxes

## Dependencies
- ROS2 Humble
- Gazebo 11
- Nav2
- SLAM Toolbox
- TurtleBot3
- OpenCV 4.8.1
- NumPy 1.26.4

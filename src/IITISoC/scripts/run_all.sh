#!/bin/bash

echo "Starting IITISoC Full System..."

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_IP=127.0.0.1
export ROS_DOMAIN_ID=30

# Terminal 1 - Gazebo + Robots
gnome-terminal --title="Gazebo" -- bash -c "
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_IP=127.0.0.1
export ROS_DOMAIN_ID=30
ros2 launch ~/ros2_ws/src/IITISoC/launch/multi_robot.launch.py
exec bash"

sleep 15

# Terminal 2 - Detection Node
gnome-terminal --title="ArUco Detection" -- bash -c "
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/IITISoC/scripts/aruco_detector.py
exec bash"

sleep 3

# Terminal 3 - Robot 1 Patrol
gnome-terminal --title="Robot1 Patrol" -- bash -c "
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/IITISoC/scripts/patrol_robot1.py
exec bash"

sleep 3

# Terminal 4 - Robot 2 Patrol
gnome-terminal --title="Robot2 Patrol" -- bash -c "
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/IITISoC/scripts/patrol_robot2.py
exec bash"

echo "All systems launched!"
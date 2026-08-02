# IITISoC 2026 — Autonomous Waste Detection and Multi-Agent Task Allocation

## Project Description

This project implements a full autonomous waste detection and collection system using **three TurtleBot3 Waffle robots** in a simulated 20m × 20m, three-room warehouse environment (Gazebo). Each robot detects ArUco markers (representing waste items) through its onboard camera, estimates the 3D world position of each marker, and reports it to a shared database node. A task allocator then assigns each confirmed waste item to the best available robot using the Hungarian algorithm, and each robot autonomously navigates to and covers its assigned zone using Nav2 and a frontier-based coverage strategy.

This README covers the **complete final system**: perception/detection, shared database, multi-agent task allocation, navigation, and coverage. A bonus mechanical CAD design (Fusion 360) was also completed as a real-world deployable counterpart of the simulated robot.

---

## Team

| Name | Role |
|---|---|
| Shrawani | Perception & ArUco Detection, TF2, Design Analysis, Database Node (initial creation) |
| Aashlesha | Simulation & Navigation, SLAM, Task Allocation, Mechanical CAD Design, Database Node (improvements & integration) |

*(Full module-by-module contribution percentages are listed in the final report.)*

---

## Folder Structure

```
ros2_ws/src/
├── IITISoC/
│   ├── config/
│   │   ├── nav2_params_robot1.yaml
│   │   ├── nav2_params_robot2.yaml
│   │   ├── nav2_params_robot3.yaml
│   │   ├── to.rviz
│   │   ├── warehouse_map.pgm
│   │   └── warehouse_map.yaml
│   ├── iitisoc/
│   │   ├── __init__.py
│   │   └── aruco_detector.py                 # Main detection node
│   ├── launch/
│   │   ├── aruco_detection.launch.py         # Launches ArUco detector for all 3 robots
│   │   ├── multi_robot.launch.py             # Launches Gazebo + 3 robots
│   │   └── nav2.launch.py                    # Launches Nav2 stack for all 3 robots
│   ├── maps/
│   │   ├── warehouse_map.pgm
│   │   └── warehouse_map.yaml
│   ├── models/
│   │   ├── aruco_marker_0 … aruco_marker_11/ # 12 marker models (IDs 0–11)
│   │   │   ├── materials/scripts/aruco_marker_<id>.material
│   │   │   ├── materials/textures/aruco_marker_<id>.png
│   │   │   ├── model.config
│   │   │   └── model.sdf
│   │   ├── turtlebot3_waffle_robot1/model.sdf
│   │   ├── turtlebot3_waffle_robot2/model.sdf
│   │   ├── turtlebot3_waffle_robot3/model.sdf
│   │   └── warehouse_labels/
│   │       ├── materials/scripts/labels.material
│   │       ├── materials/textures/ (label_cupboard.png, label_loading_area.png,
│   │       │                        label_machine1.png, label_machine2.png, label_rack.png)
│   │       ├── model.config
│   │       └── model.sdf
│   ├── resource/iitisoc
│   ├── scripts/
│   ├── urdf/
│   │   ├── turtlebot3_waffle_fixed.urdf
│   ├── worlds/
│   │   ├── warehouse-world(box).world
│   │   └── warehouse_world.world             # Final 20m x 20m, 3-room warehouse
│   ├── .gitignore
│   ├── README.md
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── coverage_frontier_interfaces/
│   ├── srv/
│   │   ├── GetRoomStatus.srv
│   │   ├── QueryNearestUnseen.srv
│   │   └── ReserveDoor.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
├── coverage_frontier_pkg/
│   ├── config/rooms.yaml
│   ├── coverage_frontier_pkg/
│   │   ├── __init__.py
│   │   ├── coordinator_node.py
│   │   ├── coverage_grid_server.py
│   │   ├── door_reservation_server.py
│   │   └── robot_coverage_node.py
│   ├── launch/coverage_frontier.launch.py
│   ├── resource/coverage_frontier_pkg
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── task_allocator/
│   ├── launch/task_allocation.launch.py
│   ├── resource/task_allocator
│   ├── task_allocator/
│   │   ├── __init__.py
│   │   ├── allocator_node.py
│   │   └── goal_sender_node.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── task_allocator_msgs/
│   ├── msg/
│   │   ├── RobotState.msg
│   │   ├── Target.msg
│   │   └── TaskAssignment.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
├── waste_database/
│   ├── dashboard/waste_dashboard.html         # Live rosbridge-powered web dashboard
│   ├── launch/
│   │   ├── aruco_database_bridges.launch.py
│   │   ├── robot_state_bridges.launch.py
│   │   ├── task_managers.launch.py
│   │   └── waste_database.launch.py
│   ├── resource/waste_database
│   ├── waste_database/
│   │   ├── __init__.py
│   │   ├── aruco_database_bridge_node.py
│   │   ├── robot_state_bridge_node.py
│   │   ├── task_manager_node.py
│   │   └── waste_database_node.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
└── waste_interfaces/
    ├── msg/
    │   ├── RobotState.msg
    │   ├── RobotStateArray.msg
    │   ├── TaskAssignment.msg
    │   ├── TaskAssignmentArray.msg
    │   ├── WasteTarget.msg
    │   └── WasteTargetArray.msg
    ├── srv/
    │   ├── RegisterDetection.srv
    │   ├── ReportTaskStatus.srv
    │   └── RequestTask.srv
    ├── CMakeLists.txt
    └── package.xml
```

---

## Dependencies

**ROS 2 and System:**
```bash
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-description
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-vision-opencv
sudo apt install python3-cv-bridge
sudo apt install ros-humble-rosbridge-server   # powers the live web dashboard
```

**Python:**
```bash
pip3 install "numpy<2"
pip3 install "opencv-contrib-python==4.8.1.78"
pip3 install scipy   # required by task_allocator for the Hungarian algorithm
```

> **Note:** NumPy must stay below 2.0 and OpenCV at 4.8.1.78 for cv_bridge compatibility with ROS 2 Humble.

---

## Setup Instructions

**1. Clone the repository and build:**
```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**2. Set environment variables (add to ~/.bashrc):**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
echo "export GAZEBO_IP=127.0.0.1" >> ~/.bashrc
source ~/.bashrc
```

**3. Place ArUco marker models in Gazebo:**

Each marker (ID 0–11, 12 markers total) needs a model folder at `~/.gazebo/models/aruco_marker_<id>/` with the following structure:

```
aruco_marker_0/
├── model.config
├── model.sdf
└── materials/
    ├── scripts/
    │   └── aruco_marker_0.material
    └── textures/
        └── aruco_marker_0.png
```

Marker images must be generated with a white quiet zone border:
```python
import cv2
import cv2.aruco as aruco
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
for i in range(12):
    marker = aruco.generateImageMarker(aruco_dict, i, 700)
    bordered = cv2.copyMakeBorder(marker, 50, 50, 50, 50,
                                   cv2.BORDER_CONSTANT, value=255)
    cv2.imwrite(f'waste_target_id_{i}.png', bordered)
```

---

## How to Run the Demo

Each of the following runs in its own terminal, in order:

**Terminal 1 — Launch Gazebo world with all 3 robots:**
```bash
ros2 launch iitisoc multi_robot.launch.py
```

**Terminal 2 — Launch Nav2 for all 3 robots:**
```bash
ros2 launch iitisoc nav2.launch.py
```

**Terminal 3 — Launch the shared waste database:**
```bash
ros2 launch waste_database waste_database.launch.py
```

**Terminal 4 — Launch ArUco detection for all 3 robots:**
```bash
ros2 launch iitisoc aruco_detection.launch.py
```

**Terminal 5 — Launch the coverage/exploration system:**
```bash
ros2 launch coverage_frontier_pkg coverage_frontier.launch.py \
  map_pgm_path:=/home/<user>/ros2_ws/src/IITISoC/config/warehouse_map.pgm \
  map_yaml_path:=/home/<user>/ros2_ws/src/IITISoC/config/warehouse_map.yaml
```

**Terminal 6 — Launch the multi-agent task allocator:**
```bash
ros2 launch task_allocator task_allocation.launch.py
```

**Terminal 7 (optional) — RViz visualization:**
```bash
rviz2 -d /home/<user>/ros2_ws/src/IITISoC/config/to.rviz
```

> Replace `<user>` with your actual home directory username in the paths above.

---

## Detection Output

**Terminal output on detection:**
```
[aruco_detector]: ID: 1 | Distance: 1.24m | x=0.05 y=0.12 z=1.23
```

**Perception (per robot):**

| Topic | Message Type | Purpose |
|---|---|---|
| `/robot{N}/camera/image_aruco_annotated` | `sensor_msgs/Image` | Annotated camera feed |
| `/aruco_detections/pose` | `geometry_msgs/PoseStamped` | Marker position for Nav2 |
| `/aruco_detections/log` | `std_msgs/String` | JSON detection log for database |

**Shared Database & Task Allocation (`task_allocator_node`):**

| Topic | Type | Direction |
|---|---|---|
| `/waste_targets` | Target info | Subscribed |
| `/robot_states` | Robot status info | Subscribed / Published (per robot) |
| `/task_assignments` | Final assignment (Hungarian algorithm output) | Published |

**Task Execution (`robot_agent_node`, per robot):**

| Topic / Action | Type | Purpose |
|---|---|---|
| `/robot{N}/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Sent when a task is assigned |
| `/robot{N}/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Source for `/robot_states` position |

**Coverage (`coverage_frontier_pkg`) — services, not topics:**

| Service | Purpose |
|---|---|
| `QueryNearestUnseen` | Robot asks for the nearest unseen cell in its room |
| `GetRoomStatus` | Checks how much of a room is still uncovered |
| `ReserveDoor` | Locks a door so only one robot crosses at a time |

Coverage also consumes each robot's `/robot{N}/amcl_pose` to mark cells as seen — no dedicated coverage topics beyond that.

**Navigation (Nav2, per robot):**

| Topic | Type | Purpose |
|---|---|---|
| `/robot{N}/cmd_vel`, `/robot{N}/cmd_vel_nav` | `geometry_msgs/Twist` | Velocity commands |
| `/robot{N}/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/robot{N}/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/robot{N}/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL initial pose (auto-set at launch) |
| `/robot{N}/global_costmap/costmap`, `/robot{N}/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Costmaps |
| `/map` | `nav_msgs/OccupancyGrid` | Shared SLAM map |

**Verify topics live:**
```bash
ros2 topic echo /aruco_detections/log
ros2 topic echo /waste_targets
ros2 topic echo /task_assignments
```

*(Full schema, service definitions, and message formats are documented in the final report — Sections 10 and 11.)*

---

## Detection Specifications

| Parameter | Value |
|---|---|
| ArUco Dictionary | DICT_4X4_50 |
| Marker IDs in use | 0 to 11 (12 total) |
| Marker size (Gazebo) | 0.5m x 0.5m |
| Detection range | 0.5m to 2.0m |
| Camera topic | `/robot{N}/camera/image_raw` |
| Pose estimation method | PnP (Perspective n Point) |

---

## Gazebo Simulation

A custom **20m × 20m, three-room warehouse** built in Gazebo 11 with an L-shaped (non-rectangular) footprint. The three rooms — a storage hall, a processing area, and a loading bay — are connected by three corridors, giving robots multiple route options and creating genuine navigation challenges (dead ends, narrow doorways, deliberate room entry). Each robot spawns in its own zone: robot1 → warehouse, robot2 → storage, robot3 → office.

---

## Navigation

Autonomous navigation is handled via **Nav2** with **AMCL** localization on a pre-built SLAM map, using the RegulatedPurePursuitController as the local controller for all three robots. Initial poses are set automatically at launch to match each robot's spawn position, with staggered lifecycle-manager startup to avoid bond timeouts on a resource-constrained VM.

---

## Coverage Strategy

Each robot systematically covers its assigned room using a **coverage-frontier** approach — repeatedly querying the nearest unseen cell in its room (not a pre-computed path) and navigating there via Nav2. A shared coverage grid prevents redundant coverage across robots, and robots automatically reallocate to the next-least-covered room once their own room is finished. Full details, known limitations, and quantitative coverage figures are documented in the final report.

---

## Multi-Agent Task Allocation

Waste items confirmed by the shared database are assigned to robots using the **Hungarian algorithm** (`scipy.optimize.linear_sum_assignment`), which finds the globally optimal robot-to-target matching based on distance, target priority, and robot workload — rather than simply assigning each target to the nearest robot one at a time. Full algorithm details, conflict resolution, and scalability discussion are documented in the final report.

---

## Shared Database

A centralized database (`task_allocator_node`) tracks all detected waste items, robot states, and task assignments using in-memory Python dictionaries, communicating with the rest of the system entirely over ROS2 topics. A live web dashboard (`waste_database/dashboard/waste_dashboard.html`), served via `rosbridge_server`, visualizes real-time system status. Full schema and workflow details are documented in the final report.

---

## Mechanical CAD Design (Bonus)

A full physical robot platform was designed from scratch in **Autodesk Fusion 360** as the real-world counterpart of the simulated TurtleBot3 Waffle, including chassis, sensor mast, motor mounts, and mass/centre-of-gravity analysis. Full specifications are documented in the final report.

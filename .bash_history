ros2 run demo_nodes_py listener
ros2 --version
ros2 pkg list
ros2 pkg xml ros2cli | grep version
ros2 --version
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-ros2-control -y
gazebo
sudo apt install ros-humble-turtlebot3* -y
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
ros2 doctor
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-slam-toolbox -y
sudo apt install ros-humble-apriltag-ros -y
sudo apt install python3-opencv -y
sudo apt install python3-pip -y
pip3 install opencv-contrib-python --break-system-packages
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update
sudo apt install ros-humble-rqt* -y
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
sudo snap install code --classic
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
exit
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop-full -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 --version
sudo apt update
sudo apt install ros-humble-desktop-full -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 --version
echo $ROS_DISTRO
apt list --installed | grep ros-core
ros2 -h
source /opt/ros/humble/setup.bash
ros2 doctor
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ros2 run demo_nodes_cpp talker
exit
sudo sh ./VBoxLinuxAdditions.run
[200~git config --global user.name "YourGitHubUsername"
rqt_graph
ros2 run demo_nodes_cpp
ros2 run demo_nodes_cpp talker
ros2 run turtlesim turtlesim_node
ros2 run demo_nodes_cpp listener
ros2 run turtlesim turtle_teleop_key

# RRT-Path-Planning-Application-My-own-world---ROS2-
Autonomous robot simulation of solving mazes using Q-Learning and RRT applications in ROS 2 and Gazebo environments.

# 1. Create Workspacei
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Download this Project
git clone https://github.com/username/rl_maze_nav.git

# 3. Compile
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 4. Describe the robot model
export TURTLEBOT3_MODEL=burger

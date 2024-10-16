#!/bin/bash
# test_slam.sh

# Source ROS and workspace setup
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Launch TurtleBot in MyWorld2.world using xterm
echo "Launching turtlebot_world with MyWorld2.world..."
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld2.world" &

sleep 5 # Give Gazebo time to load

# Launch SLAM with gmapping using xterm
echo "Launching SLAM (gmapping)..."
xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 5 # Allow SLAM to initialize


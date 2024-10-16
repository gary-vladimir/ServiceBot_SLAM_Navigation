#!/bin/bash
# test_navigation.sh

# Source ROS and workspace setup
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Launch TurtleBot in MyWorld2.world using xterm
echo "Launching turtlebot_world with MyWorld2.world..."
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld2.world" &

sleep 5 # Give Gazebo time to load

# Launch AMCL for localization using xterm
echo "Launching AMCL for localization..."
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &

sleep 5 # Allow AMCL to initialize

# Launch RViz for map visualization using xterm
echo "Launching RViz..."
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &


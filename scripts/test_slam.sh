#!/bin/bash
# test_slam.sh

# Source ROS and workspace setup
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Launch TurtleBot in MyWorld2.world
echo "Launching turtlebot_world with World2.world..."
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(find my_robot)/worlds/MyWorld2.world &

sleep 5 # Give Gazebo time to load

# Launch SLAM with gmapping
echo "Launching SLAM (gmapping)..."
roslaunch turtlebot_gazebo gmapping_demo.launch &

sleep 5 # Allow SLAM to initialize

# Launch RViz for map visualization
echo "Launching RViz..."
roslaunch turtlebot_rviz_launchers view_navigation.launch &

sleep 5 # Wait for RViz to load

# Launch teleop for manual control
echo "Launching teleop for manual control..."
rosrun teleop_twist_keyboard teleop_twist_keyboard.py &


#!/bin/bash
# add_marker.sh

# Source ROS and workspace setup
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Launch TurtleBot in MyWorld2.world using xterm
echo "Launching turtlebot_world with MyWorld2.world..."
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld2.world" &

sleep 5 # Give Gazebo time to load

# Launch AMCL for localization using xterm
echo "Launching AMCL for localization..."
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/map.yaml initial_pose_x:=0 initial_pose_y:=0 initial_pose_a:=-1.5708" &

sleep 5 # Allow AMCL to initialize

# Launch RViz for map visualization using xterm
echo "Launching RViz..."
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5 # Give RViz time to load

# Run the add_markers node to display the marker
echo "Running add_markers node..."
xterm -hold -e "rosrun add_markers add_markers" &


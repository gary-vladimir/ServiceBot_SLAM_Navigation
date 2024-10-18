#!/bin/bash
# home_service.sh

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

# Launch RViz with custom configuration using xterm
echo "Launching RViz with custom configuration..."
xterm -e "rosrun rviz rviz -d ~/catkin_ws/src/rvizConfig/navigation_with_marker.rviz" &

sleep 5 # Allow RViz to initialize

# Launch add_markers_with_robot node using xterm
echo "Launching add_markers_with_robot node..."
xterm -e "rosrun add_markers add_markers_with_robot" &

sleep 5 # Allow add_markers to initialize

# Launch pick_objects node using xterm
echo "Launching pick_objects node..."
xterm -hold -e "rosrun pick_objects pick_objects" &


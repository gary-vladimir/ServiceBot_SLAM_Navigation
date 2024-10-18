# Home Service Robot Project

## Overview

This project implements a home service robot simulation using ROS Noetic, Gazebo, and RViz. The robot is capable of autonomous navigation and manipulation in a simulated environment, where it can pick up and drop off objects. The robot performs localization using AMCL, navigation with `move_base`, and visualizes markers for virtual objects using RViz markers.

The project demonstrates key aspects of robotics including SLAM, autonomous navigation, and simulated object manipulation.

---

## Features

- Autonomous Navigation: The robot autonomously navigates through a predefined map, moving between pickup and drop-off zones.
- AMCL Localization: The robot uses the Adaptive Monte Carlo Localization (AMCL) algorithm to localize itself within the environment.
- Move Base Navigation: Uses `move_base` to generate and execute navigation goals.
- RViz Markers: RViz markers represent virtual objects at pickup and drop-off zones.
- Odometry Integration: The robot's position is tracked in real-time using odometry data.
- Kobuki Base Simulation: The project utilizes the Kobuki base for the TurtleBot simulation in Gazebo.

---

## Packages and Nodes

1. **my_robot**:
   - Contains world files, and configuration for the robot in Gazebo.
   - Launches the robot in a custom world file, `MyWorld2.world`.

2. **add_markers**:
   - Displays virtual objects in RViz.
   - The `add_markers_with_robot` node subscribes to odometry data, allowing the robot to "pick up" and "drop off" virtual objects at predefined locations.
   
3. **pick_objects**:
   - The `pick_objects` node sends navigation goals to the robot, simulating the behavior of moving to a pickup location, waiting, and then moving to a drop-off location.

4. **turtlebot_gazebo**:
   - Simulates the TurtleBot using Gazebo.
   - Integrates the Kobuki base, sensors, and navigation for the robot.

5. **turtlebot_navigation**:
   - Provides the necessary components for robot localization and navigation, including AMCL, global and local cost maps.

---

## Installation

### Prerequisites

- ROS Noetic (or the ROS version you're using)
- Gazebo 11.14.0
- RViz

### Clone the Repository

git clone https://github.com/gary-vladimir/ServiceBot_SLAM_Navigation.git

### Build the Workspace

Navigate to your ROS workspace and build:

cd ~/catkin_ws
catkin_make
source devel/setup.bash

---

## Usage

### 1. Launch the Simulation

Run the following command to start Gazebo and load the robot in the `MyWorld2` environment:

roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld2.world

### 2. Start Localization with AMCL

Launch the AMCL localization:

roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/map.yaml initial_pose_x:=0 initial_pose_y:=0 initial_pose_a:=-1.5708

### 3. Visualize in RViz

To visualize the robot, navigation goals, and markers in RViz, launch the custom RViz configuration:

roslaunch turtlebot_rviz_launchers view_navigation.launch config_file:=$(rospack find rvizConfig)/navigation_with_marker.rviz

### 4. Run the Pick and Place Behavior

Now, run the node to command the robot to move to the pickup and drop-off zones:

rosrun pick_objects pick_objects

### 5. Display Markers in RViz

Run the node to display markers for the pickup and drop-off zones, which will be updated based on the robot’s position:

rosrun add_markers add_markers_with_robot

---

## Additional Scripts

### Home Service Robot Script

For ease of use, a script `home_service.sh` is provided to launch all components at once:

cd ~/catkin_ws/src/scripts
./home_service.sh

This script launches:
- Gazebo with the TurtleBot in `MyWorld2`
- AMCL for localization
- RViz with a custom configuration
- `pick_objects` node for navigation goals
- `add_markers_with_robot` for displaying virtual objects in RViz

---

## Results

### Robot in Simulation Environment

![Alt text](./result.gif)

*The robot navigating in the simulated environment.*

---

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## Contact

For any questions or collaboration requests, please contact me at gary@garybricks.com.



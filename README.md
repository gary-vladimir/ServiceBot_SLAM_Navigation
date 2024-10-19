
# Home Service Robot Project

## Overview

This project demonstrates a home service robot capable of autonomous navigation and object manipulation in a simulated environment. Using ROS Noetic, Gazebo, and RViz, the robot explores its surroundings, builds a map, localizes itself, and navigates to specific locations using predefined goals. Key functionalities include Simultaneous Localization and Mapping (SLAM), localization via Adaptive Monte Carlo Localization (AMCL), and path planning with Dijkstra’s algorithm.

---

## How It Works

The mobile robot first navigates through the environment using its laser scanner to generate a static map through SLAM. Once the map is created, the robot uses odometry and laser data to localize itself using **Adaptive Monte Carlo Localization (AMCL)**. When a navigation goal is provided, the robot plans its path using **Dijkstra’s algorithm**, ensuring it finds the optimal route to the goal. Throughout this process, the robot visualizes virtual objects at pickup and drop-off locations in RViz and autonomously completes navigation tasks.

---

## Core Packages and Algorithms

### 1. **pick_objects** (Navigation Command)

The `pick_objects` package controls the robot’s movement between designated pickup and drop-off locations. It sends navigation goals to the **move_base** node, which utilizes Dijkstra’s algorithm to plan and execute the navigation trajectory. The `pick_objects` node ensures the robot reaches both the pickup and drop-off zones autonomously.

- **Functionality**: Sends predefined or dynamically assigned navigation goals to the robot for autonomous movement.

### 2. **add_markers** (RViz Markers)

The `add_markers` package is responsible for displaying virtual objects in RViz. The `add_markers_with_robot` node listens to the robot's odometry data to determine when the robot has reached the pickup or drop-off zones, updating the markers accordingly to simulate object pickup and drop-off actions.

- **Functionality**: Dynamically manages RViz markers that represent objects being picked up or dropped off by the robot.

### 3. **my_robot** (Custom World and Robot Configuration)

The `my_robot` package contains the configuration files, URDF robot model, and the Gazebo world used in the simulation. The `MyWorld2.world` file defines the environment in which the robot operates, while the URDF defines the physical characteristics of the robot, including its sensors (such as the laser scanner and odometry).

- **Functionality**: Sets up the robot model and the simulated environment for Gazebo.

### 4. **slam_gmapping** (Simultaneous Localization and Mapping - SLAM)

The **slam_gmapping** package allows the robot to perform SLAM, building a 2D occupancy grid map while exploring the environment. The **GMapping** algorithm uses a particle filter to estimate the robot's position while constructing the map based on laser scan data.

- **Algorithm**: **GMapping**, a particle filter-based SLAM algorithm that creates a map while localizing the robot in real-time.

- **Functionality**: Builds a static map of the environment while the robot navigates, using laser data and odometry.

### 5. **turtlebot** (Robot Simulation)

The `turtlebot` package simulates the TurtleBot platform, which includes the Kobuki base, laser scanner, and odometry sensors. This package integrates the robot's hardware and sensors into the simulation environment, enabling interaction with Gazebo and RViz for visualization and navigation.

- **Functionality**: Simulates the physical robot and its sensors for use in the Gazebo environment.

### 6. **turtlebot_navigation** (Localization and Navigation)

The `turtlebot_navigation` package integrates the **move_base**, **amcl**, and costmap functionality to provide the robot with autonomous navigation capabilities. It includes the configuration for both the **global planner**, which plans the route from the start to the goal using **Dijkstra’s algorithm**, and the **local planner**, which dynamically adjusts the path to avoid obstacles in real-time.

- **Algorithms**:
  - **AMCL**: Adaptive Monte Carlo Localization for probabilistic localization using laser scans and odometry data.
  - **Dijkstra’s Algorithm**: A graph-based search algorithm used for path planning. It guarantees the robot finds the optimal path to the navigation goal based on costmaps that represent obstacles and free space.

- **Functionality**: Provides localization and path planning capabilities, enabling the robot to navigate autonomously to given goals while avoiding obstacles.

---

## Description of Project Components

1. **Gazebo World and Mobile Robot**: The robot operates in a custom Gazebo environment (`MyWorld2.world`), which is designed for the simulation of autonomous navigation.

2. **ROS Packages**:
   - **slam_gmapping**: Used to generate a static map of the environment.
   - **amcl**: Used for probabilistic localization once the map is created.
   - **move_base**: Handles path planning and navigation.
   - **pick_objects**: Sends navigation goals to move the robot to pickup and drop-off locations.
   - **add_markers**: Manages RViz visualization of virtual objects.

---

## Installation

### Prerequisites

- ROS Noetic
- Gazebo 11.14.0
- RViz

### Clone the Repository

```bash
git clone https://github.com/gary-vladimir/ServiceBot_SLAM_Navigation.git
```

### Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Usage

### 1. Launch the Simulation

```bash
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld2.world
```

### 2. Start SLAM (for Mapping)

```bash
roslaunch turtlebot_gazebo gmapping_demo.launch
```

### 3. Start Localization with AMCL (for Pre-built Map)

```bash
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/map.yaml initial_pose_x:=0 initial_pose_y:=0 initial_pose_a:=-1.5708
```

### 4. Visualize in RViz

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch config_file:=$(rospack find rvizConfig)/navigation_with_marker.rviz
```

### 5. Run the Pick and Place Behavior

```bash
rosrun pick_objects pick_objects
```

### 6. Display Markers in RViz

```bash
rosrun add_markers add_markers_with_robot
```

---

Here’s an improved "Scripts" section based on the information you provided:

---

## Scripts

To simplify launching different components of the home service robot project, several bash scripts are provided. These scripts automate the process of starting the necessary ROS nodes and configuring the environment for various tasks such as SLAM, navigation, and marker manipulation in RViz.

### Available Scripts

1. **add_marker.sh**  
   This script launches the robot in the Gazebo world, initializes AMCL for localization, opens RViz for visualization, and runs the `add_markers` node to display virtual objects in RViz. Use this to simulate object pickup and drop-off visualizations.
   
   ```bash
   ./add_marker.sh
   ```

2. **pick_objects.sh**  
   This script launches the robot, runs AMCL for localization, opens RViz for map visualization, and starts the `pick_objects` node, which simulates the robot navigating between pickup and drop-off locations.
   
   ```bash
   ./pick_objects.sh
   ```

3. **test_slam.sh**  
   This script runs the SLAM process, launching the robot in Gazebo, initializing the SLAM system with **GMapping**, and visualizing the generated map in RViz. It also opens teleoperation for manual control via keyboard, allowing you to manually drive the robot and generate the map.
   
   ```bash
   ./test_slam.sh
   ```

4. **test_navigation.sh**  
   This script is used to test navigation and localization after a map has been generated. It launches the robot in Gazebo, runs AMCL for localization, and opens RViz to visualize the robot’s position and navigation goals.
   
   ```bash
   ./test_navigation.sh
   ```

5. **home_service.sh**  
   This is a comprehensive script that automates the entire home service robot operation. It launches the robot, initializes AMCL for localization, opens a customized RViz configuration for navigation with markers, runs the `add_markers_with_robot` node to visualize objects in RViz, and starts the `pick_objects` node to simulate the robot autonomously navigating between pickup and drop-off zones.

   ```bash
   ./home_service.sh
   ```

### Script Breakdown

- **ROS and Workspace Setup**: Each script starts by sourcing the ROS environment and the workspace setup to ensure all necessary environment variables are configured correctly.
  
  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  ```

- **Gazebo World and AMCL Localization**: All scripts launch the robot in `MyWorld2.world` and use **AMCL** for localization, which enables the robot to localize itself on a pre-built map. This is crucial for tasks like navigation and marker handling.

  ```bash
  roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_robot)/worlds/MyWorld2.world
  roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/map.yaml initial_pose_x:=0 initial_pose_y:=0 initial_pose_a:=-1.5708
  ```

- **RViz Launch**: Each script also launches **RViz** for visualizing the robot’s environment, position, and navigation goals, providing a clear overview of the robot’s tasks.

  ```bash
  roslaunch turtlebot_rviz_launchers view_navigation.launch
  ```

- **Node Execution**: Depending on the specific task, the scripts run different nodes. For instance, `add_marker.sh` launches the `add_markers` node to handle virtual object visualization, while `pick_objects.sh` launches the `pick_objects` node for task navigation.


---

## Results

### Robot in Simulation Environment

![Alt text](./result.gif)

---

## License

This project is licensed under the MIT License. See the LICENSE file for details.

---

## Contact

For any questions or collaboration requests, contact me at gary@garybricks.com.

---

This README now focuses on the relevant packages and provides clear descriptions of the algorithms used for SLAM, localization, and navigation, including Dijkstra’s algorithm for path planning.

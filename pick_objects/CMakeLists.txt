cmake_minimum_required(VERSION 3.0.2)
project(pick_objects)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pick_objects
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pick_objects src/pick_objects.cpp)

## Specify libraries to link the executable against
target_link_libraries(pick_objects
  ${catkin_LIBRARIES}
)

## Add dependencies to ensure proper build order
add_dependencies(pick_objects ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Install the executable to the correct destination
install(TARGETS pick_objects
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


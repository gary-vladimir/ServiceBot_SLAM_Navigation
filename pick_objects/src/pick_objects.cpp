#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  

  // ********** Move to Pick-Up Zone **********
  ROS_INFO("Robot moving to pick-up zone");

  // Set pick-up location
  goal.target_pose.pose.position.x = 0.6732583699368697;
  goal.target_pose.pose.position.y = 0.7174597094373839;
  goal.target_pose.pose.orientation.z = 0.21298343118087062;
  goal.target_pose.pose.orientation.w = 0.9770558111195201;

  // Send the goal
  ac.sendGoal(goal);

  // Wait for the result
  ac.waitForResult();

  // Check if the robot reached the pick-up zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached pick-up zone, picking up imaginary object");
  else{
    ROS_INFO("Failed to reach pick-up zone");
    return 1;
  }

	// Simulate pick-up time
  ros::Duration(5.0).sleep();
  
  // ********** Move to Drop-off Zone **********
  ROS_INFO("Robot now moving to drop-off zone");

  // Set drop-off location
  goal.target_pose.pose.position.x = -0.6052637241814067;
  goal.target_pose.pose.position.y = -1.093148875567051;
  goal.target_pose.pose.orientation.z = 0.036281418144198836;
  goal.target_pose.pose.orientation.w = 0.9993416126117464;

  // Send the goal
  ac.sendGoal(goal);

  // Wait for the result
  ac.waitForResult();

  // Check if the robot reached the drop-off zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Dropped the object successfully");
  else
    ROS_INFO("Failed to reach drop-off zone");
    
  return 0;
}

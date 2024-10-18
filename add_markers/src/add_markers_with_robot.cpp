#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

// Coordinates for pickup and drop-off zones
float pickup_x = 0.6732583699368697;
float pickup_y = 0.7174597094373839;

float dropoff_x = -0.6052637241814067;
float dropoff_y = -1.093148875567051;

bool reached_pickup = false;
bool reached_dropoff = false;
ros::Publisher marker_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  float robot_x = msg->pose.pose.position.x;
  float robot_y = msg->pose.pose.position.y;

  // Function to calculate the distance between robot and goal
  auto distance_to_goal = [](float x1, float y1, float x2, float y2) -> float {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  };

  float distance_to_pickup = distance_to_goal(robot_x, robot_y, pickup_x, pickup_y);
  float distance_to_dropoff = distance_to_goal(robot_x, robot_y, dropoff_x, dropoff_y);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers_with_robot";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();  // Marker remains until deleted

  // If robot reaches the pickup zone and marker is still there
  if (distance_to_pickup < 0.5 && !reached_pickup) {
    ROS_INFO("Robot reached the pickup zone, hiding marker...");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    reached_pickup = true;
    ros::Duration(5.0).sleep();  // Simulate picking up the object
  }

  // If robot reaches the drop-off zone and marker is hidden
  if (distance_to_dropoff < 0.5 && reached_pickup && !reached_dropoff) {
    ROS_INFO("Robot reached the drop-off zone, showing marker...");
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = dropoff_x;
    marker.pose.position.y = dropoff_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker_pub.publish(marker);
    reached_dropoff = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  // Publisher to publish marker
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Subscriber to odometry to track the robot's position
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

  // Initially, publish the marker at the pickup zone
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pickup_x;
  marker.pose.position.y = pickup_y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  ROS_INFO("Publishing the marker at the pickup zone...");
  marker_pub.publish(marker);

  // Keep the node alive and responsive to the odometry callback
  ros::spin();

  return 0;
}


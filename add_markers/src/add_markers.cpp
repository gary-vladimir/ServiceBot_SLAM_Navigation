#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Define the pickup and drop-off zone coordinates
  float pickup_x = 0.6732583699368697;
  float pickup_y = 0.7174597094373839;

  float dropoff_x = -0.6052637241814067;
  float dropoff_y = -1.093148875567051;

  // Define a cube shape
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Wait for a subscriber to connect
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Waiting for a subscriber to connect to the marker...");
    sleep(1);
  }

  // Set up the marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker (size)
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color (opaque green)
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();  // This makes the marker last indefinitely

  // Step 1: Publish the marker at the pickup zone
  marker.pose.position.x = pickup_x;
  marker.pose.position.y = pickup_y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  
  ROS_INFO("Publishing the marker at the pickup zone...");
  marker_pub.publish(marker);

  // Step 2: Wait for 5 seconds
  ros::Duration(5.0).sleep();

  // Step 3: Hide the marker (DELETE action)
  marker.action = visualization_msgs::Marker::DELETE;
  ROS_INFO("Hiding the marker...");
  marker_pub.publish(marker);

  // Step 4: Wait for 5 seconds
  ros::Duration(5.0).sleep();

  // Step 5: Reset the marker for the drop-off zone
  marker.header.stamp = ros::Time::now();  // Reset timestamp
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = dropoff_x;
  marker.pose.position.y = dropoff_y;

  // Ensure the marker remains visible indefinitely
  marker.lifetime = ros::Duration(0);  // Set marker to last indefinitely

  ROS_INFO("Publishing the marker at the drop-off zone...");
  marker_pub.publish(marker);

  // ** Extra: Spin to keep node alive to ensure marker stays visible **
  ros::Rate r(10);
  while (ros::ok())
  {
    marker_pub.publish(marker);  // Re-publish to ensure it's updated in RViz
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


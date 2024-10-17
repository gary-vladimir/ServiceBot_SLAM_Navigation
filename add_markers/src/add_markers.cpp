#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");  // Modified node name
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set the shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Create a single marker in the pick-up zone
  visualization_msgs::Marker marker;
  
  // Set the frame ID and timestamp
  marker.header.frame_id = "map";  // Modify the frame_id as "map"
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type to be a cube
  marker.type = shape;

  // Set the marker action. Option: ADD
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker at the pick-up zone
  marker.pose.position.x = 0.6732583699368697;
  marker.pose.position.y = 0.7174597094373839;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.21298343118087062;
  marker.pose.orientation.w = 0.9770558111195201;

  // Set the scale of the marker (size)
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();  // Marker will last indefinitely

  // Publish the marker
  while (ros::ok())
  {
    if (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);  // Publish the cube

    r.sleep();
  }
}


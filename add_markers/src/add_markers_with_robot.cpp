#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <cmath>

// Coordinates for pickup and drop-off zones
double pickup_x = 0.6732583699368697;
double pickup_y = 0.7174597094373839;

double dropoff_x = -0.6052637241814067;
double dropoff_y = -1.093148875567051;

bool reached_pickup = false;
bool reached_dropoff = false;
ros::Publisher marker_pub;
tf::TransformListener *listener;

// Function to calculate the distance between robot and goal
double distance_to_goal(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Transform robot's position from the 'odom' frame to the 'map' frame
    tf::StampedTransform transform;
    try {
        listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    double robot_x = transform.getOrigin().x();
    double robot_y = transform.getOrigin().y();

    double distance_to_pickup = distance_to_goal(robot_x, robot_y, pickup_x, pickup_y);
    double distance_to_dropoff = distance_to_goal(robot_x, robot_y, dropoff_x, dropoff_y);

    // Debugging output: print robot's position and distance to pickup/dropoff
    ROS_INFO("Robot position (in map frame): x = %f, y = %f", robot_x, robot_y);
    ROS_INFO("Distance to pickup: %f, Distance to dropoff: %f", distance_to_pickup, distance_to_dropoff);

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
    if (distance_to_pickup <= 0.9 && !reached_pickup) {
        ROS_INFO("Robot reached the pickup zone, hiding marker...");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        reached_pickup = true;
    }

    // If robot reaches the drop-off zone and marker is hidden
    if (distance_to_dropoff <= 0.9 && reached_pickup && !reached_dropoff) {
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
    ros::init(argc, argv, "add_markers_with_robot");
    ros::NodeHandle n;

    // Initialize the transform listener
    tf::TransformListener tf_listener;
    listener = &tf_listener;

    // Publisher to publish marker
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Wait for subscriber to connect before publishing initial marker
    while (marker_pub.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            return 0;
        }
        ROS_WARN_ONCE("Waiting for a subscriber to connect to the marker topic...");
        sleep(1);
    }

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

    // Subscriber to odometry to track the robot's position
    ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

    // ** Re-publish marker states to ensure updates in RViz ** 
    ros::Rate r(10);  // Loop rate of 10Hz
    while (ros::ok()) {
        // If marker is at pickup zone but needs deletion
        if (reached_pickup && !reached_dropoff) {
            marker.action = visualization_msgs::Marker::DELETE;
        }
        // If marker is to be placed at drop-off zone
        else if (reached_dropoff) {
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = dropoff_x;
            marker.pose.position.y = dropoff_y;
        }
        marker_pub.publish(marker);  // Ensure marker state is updated
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}


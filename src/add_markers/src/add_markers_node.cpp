#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

class MarkerHandler
{
private:
  /* data */
  ros::NodeHandle nh_;
  ros::Publisher marker_pub;
  ros::Subscriber odom_subscriber;
  visualization_msgs::Marker current_marker;
  std::vector<double> pickup_position = {5.9, -2.79};
  std::vector<double> dropoff_position = {2.1, 2.6};
  bool marker_picked_up = false;
  bool transport_finished = false;

  constexpr static double DIST_THRESHOLD = 0.5;

  visualization_msgs::Marker createMarker(const std::vector<double> &position, const double lifetime = -1.0);
  void publishMarker(visualization_msgs::Marker marker);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
  MarkerHandler(ros::NodeHandle *nodeHandle);
  ~MarkerHandler();
};

MarkerHandler::MarkerHandler(ros::NodeHandle *nodeHandle) : nh_(*nodeHandle)
{
  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  odom_subscriber = nh_.subscribe("/odom", 1000, &MarkerHandler::odomCallback, this);
  ros::spinOnce();
  ROS_INFO("Displaying marker at pickup location, waiting for robot to pick up");
  current_marker = createMarker(pickup_position);
  publishMarker(current_marker);
}

MarkerHandler::~MarkerHandler()
{
}

void MarkerHandler::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(transport_finished) {
    return;
  }

  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;

  double targetX = pickup_position[0];
  double targetY = pickup_position[1];

  if (marker_picked_up)
  {
    targetX = dropoff_position[0];
    targetY = dropoff_position[1];
  }
  const double dist = sqrt((x - targetX) * (x - targetX) + (y - targetY) * (y - targetY));
  if (dist <= DIST_THRESHOLD)
  {
    if (!marker_picked_up)
    {
      marker_picked_up = true;
      ROS_INFO("Pickup location reached");
      ros::Duration(5.0).sleep();
      current_marker.action = visualization_msgs::Marker::DELETE;
      publishMarker(current_marker);
      ROS_INFO("Removing pickup marker, waiting for robot to drop off");
    }
    else
    {
      transport_finished = true;
      ros::Duration(2.0).sleep();
      current_marker = createMarker(dropoff_position);
      publishMarker(current_marker);
      ROS_INFO("Dropoff location reached, showing dropped marker");
    }
  }
}

visualization_msgs::Marker MarkerHandler::createMarker(const std::vector<double> &position, const double lifetime)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "goal_marker";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  const double scale = 0.2;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = position[0];
  marker.pose.position.y = position[1];
  marker.pose.position.z = scale * 0.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  if (lifetime > 0.0)
  {
    marker.lifetime = ros::Duration(lifetime);
  }
  else
  {
    marker.lifetime = ros::Duration();
  }
  return marker;
}

void MarkerHandler::publishMarker(visualization_msgs::Marker marker)
{
  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      break;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker.header.stamp = ros::Time::now();
  marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  MarkerHandler marker_handler(&n);

  ros::spin();
  // Set our initial shape type to be a cube
}

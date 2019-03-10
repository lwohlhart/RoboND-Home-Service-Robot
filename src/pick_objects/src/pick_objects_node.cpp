#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Publisher goal_state_pub = n.advertise<std_msgs::String>("transport_goal_state", 1000);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal;

  //we'll send a pickup_goal to the robot
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();

  pickup_goal.target_pose.pose.position.x = 5.9;
  pickup_goal.target_pose.pose.position.y = -2.79;
  pickup_goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(1.2);

  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickup_goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached pickup pose");
    std_msgs::String msg;
    msg.data = "pickup_reached";
    goal_state_pub.publish(msg);
  } else
    ROS_INFO("Robot failed to move to pickup pose for some reason");
  
  ros::Duration(5.0).sleep();

  move_base_msgs::MoveBaseGoal dropoff_goal;

  //we'll send a dropoff goal to the robot
  dropoff_goal.target_pose.header.frame_id = "map";
  dropoff_goal.target_pose.header.stamp = ros::Time::now();

  dropoff_goal.target_pose.pose.position.x = 2.1;
  dropoff_goal.target_pose.pose.position.y = 2.6;
  dropoff_goal.target_pose.pose.orientation =  tf::createQuaternionMsgFromYaw(-1.4);

  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(dropoff_goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot reached the dropoff location");
    std_msgs::String msg;
    msg.data = "dropoff_reached";
    goal_state_pub.publish(msg);
  } else
    ROS_INFO("The robot failed to move to the dropoff location for some reason");

  return 0;
}


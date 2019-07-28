#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal getGoal(double  x, double  y,  
				     double qx, double qy, double qz, double qw)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map"; // With respect to 'map' frame
  goal.target_pose.header.stamp = ros::Time::now(); // Start time is 'now'
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = 0.0;  // Default to 0.0
  goal.target_pose.pose.orientation.x = qx;
  goal.target_pose.pose.orientation.y = qy;
  goal.target_pose.pose.orientation.z = qz;
  goal.target_pose.pose.orientation.w = qw;
  return goal;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_client");
  // Spin a thread by default
  MoveBaseClient ac("move_base", true);
  // Try to connect to server
  // Block until connected
  while(!ac.waitForServer(ros::Duration(5.0))) {  // waitForServer will return true if connected
						  // in given timeout
    ROS_INFO("Waiting for move_base action server to come up");
  }
  // Declare a goal argument
  move_base_msgs::MoveBaseGoal goal;
  // First, go ahead 0.6 meter without changing heading
  goal = getGoal(0.6, 0.0, 0.0, 0.0, 0.0, 1.0);
  ROS_INFO("Sending first goal");
  ac.sendGoal(goal);
  // Wait for result, if fail, cancel the goal
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reach first waypoint");
  else{
    ROS_INFO("Failed to reach forst waypoint");
    ac.cancelGoal();
  } ros::Duration(1.0).sleep();
  // Second, move 0.6 meter to the left, heading set to pi/2
  goal = getGoal(0.6, 0.6, 0.0, 0.0, 0.707, 0.707);
  ROS_INFO("Sending second goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reach second waypoint");
  else{
    ROS_INFO("Failed to reach second waypoint");
    ac.cancelGoal();
  } ros::Duration(1.0).sleep();
  // Third, return to the 0.6 meter left from the orgin with heading pi
  goal = getGoal(0.0, 0.6, 0.0, 0.0, 1.0, 0.0);
  ROS_INFO("Sending third goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reach third waypoint");
  else{
    ROS_INFO("Failed to reach third waypoint");
    ac.cancelGoal();
  } ros::Duration(1.0).sleep();
  // Finally, return to the origin with heading 3pi/2
  goal = getGoal(0.0, 0.0, 0.0, 0.0, -0.707, 0.707);
  ROS_INFO("Sending fourth goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reach fourth waypoint");
  else{
    ROS_INFO("Failed to reach fourth waypoint");
    ac.cancelGoal();
  } ros::Duration(1.0).sleep();
  // End process
  return 0;
}

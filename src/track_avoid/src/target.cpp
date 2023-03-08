#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "send_goal");

    move_base_client ac("/move_base_simple/goal", true);
    cout << "waiting for the move_base action server" << endl;
    ac.waitForResult(ros::Duration(30));
    // cout << "connected to move_base servere" << endl;
    
    move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	
    goal.target_pose.pose.position.x = -3.33;
	goal.target_pose.pose.position.y = 3.49;
	goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1;

	// cout << "Sending goal" << endl;
	ac.sendGoal(goal);
	// Wait for the action to return
	ac.waitForResult();
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO("You have reached the goal!");
	else
	    ROS_INFO("The base failed for some reason");

    // ros::spin();
    return 0;
}

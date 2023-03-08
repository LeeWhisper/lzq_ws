
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher path_pub;
ros::Publisher path_pub2;
ros::Time current_time, last_time;
ros::Time current_time2, last_time2;
nav_msgs::Path path;
nav_msgs::Path path2;
const std::string fram_id = "odom";

void slam_odo_call_back(const nav_msgs::Odometry::ConstPtr &slam_odom) // for rplidar and carto
{

    current_time = ros::Time::now();

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = slam_odom->pose.pose.position.x;
    this_pose_stamped.pose.position.y = slam_odom->pose.pose.position.y;

    this_pose_stamped.pose.orientation.x = slam_odom->pose.pose.orientation.x;
    this_pose_stamped.pose.orientation.y = slam_odom->pose.pose.orientation.y;
    this_pose_stamped.pose.orientation.z = slam_odom->pose.pose.orientation.z;
    this_pose_stamped.pose.orientation.w = slam_odom->pose.pose.orientation.w;

    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = fram_id;
    path.poses.push_back(this_pose_stamped);

    path_pub.publish(path);
    // ROS_INFO("success!");
    last_time = current_time;
}
void combined_odo_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &combined_odom)
{

    current_time2 = ros::Time::now();

    geometry_msgs::PoseStamped this_pose_stamped2;
    this_pose_stamped2.pose.position.x = combined_odom->pose.pose.position.x;
    this_pose_stamped2.pose.position.y = combined_odom->pose.pose.position.y;

    this_pose_stamped2.pose.orientation.x = combined_odom->pose.pose.orientation.x;
    this_pose_stamped2.pose.orientation.y = combined_odom->pose.pose.orientation.y;
    this_pose_stamped2.pose.orientation.z = combined_odom->pose.pose.orientation.z;
    this_pose_stamped2.pose.orientation.w = combined_odom->pose.pose.orientation.w;

    this_pose_stamped2.header.stamp = current_time2;
    this_pose_stamped2.header.frame_id = fram_id;
    path2.poses.push_back(this_pose_stamped2);

    path_pub2.publish(path2);
    // ROS_INFO("success2!");
    last_time2 = current_time2;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "showpath");
    ros::NodeHandle ph;
    path_pub = ph.advertise<nav_msgs::Path>("traj_odom", 1, true);
    path_pub2 = ph.advertise<nav_msgs::Path>("traj_ekf", 1, true);
    ros::Subscriber slam_odom_sub = ph.subscribe("/odom", 1, &slam_odo_call_back);
    ros::Subscriber combined_odom_sub = ph.subscribe("/robot_pose_ekf/odom_combined", 1, &combined_odo_call_back);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    path.header.stamp = current_time;
    path.header.frame_id = fram_id;

    current_time2 = ros::Time::now();
    last_time2 = ros::Time::now();

    path2.header.stamp = current_time2;
    path2.header.frame_id = fram_id;

    ros::spin();

    return 0;
}

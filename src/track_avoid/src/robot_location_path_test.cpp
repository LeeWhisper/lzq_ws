#include "ros/ros.h"
#include "iostream"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"


using namespace std;
std::vector<Eigen::Vector3f> p_odom_pose;
std::vector<Eigen::Vector3f> p_odom_filter_pose;
std::vector<Eigen::Vector3f> p_ukf;
std::vector<geometry_msgs::Quaternion> p_odom_quaternion;
std::vector<geometry_msgs::Quaternion> p_odom_filter_quaternion;
std::vector<geometry_msgs::Quaternion> p_ukf_quaternion;
ros::Publisher odom_path_pub;
ros::Publisher odom_filter_path_pub;
ros::Publisher ukf_path_pub;
void PublishPath(ros::Publisher& puber,
                     std::vector<Eigen::Vector3f>& path,std::vector<geometry_msgs::Quaternion>& pose_quaternion)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "/odom";

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/odom";
    for(int i = 0; i < path.size();i++)
    {
        Eigen::Vector3f traj_node = path[i];
        pose.pose.position.x = traj_node(0);
        pose.pose.position.y = traj_node(1);
        pose.pose.orientation = pose_quaternion[i];
        path_msg.poses.push_back(pose);
    }

    puber.publish(path_msg);
}

void msg_callback(const nav_msgs::OdometryConstPtr odom_msg,const nav_msgs::OdometryConstPtr odom_filter_msg)
{
    // cout<<"11111111111"<<endl;
    Eigen::Vector3f msg_pose;
    geometry_msgs::Quaternion msg_quaternion;

    msg_pose[0]=odom_msg->pose.pose.position.x;
    msg_pose[1]=odom_msg->pose.pose.position.y;
    msg_pose[2]=odom_msg->pose.pose.position.z;
    msg_quaternion=odom_msg->pose.pose.orientation;
    p_odom_pose.push_back(msg_pose);
    p_odom_quaternion.push_back(msg_quaternion);
    PublishPath(odom_path_pub,p_odom_pose,p_odom_quaternion);

    msg_pose[0]=odom_filter_msg->pose.pose.position.x;
    msg_pose[1]=odom_filter_msg->pose.pose.position.y;
    msg_pose[2]=odom_filter_msg->pose.pose.position.z;
    msg_quaternion=odom_filter_msg->pose.pose.orientation;
    p_odom_filter_pose.push_back(msg_pose);
    p_odom_filter_quaternion.push_back(msg_quaternion);
    PublishPath(odom_filter_path_pub,p_odom_filter_pose,p_odom_filter_quaternion);

    // msg_pose[0]=ukf_msg->pose.pose.position.x;
    // msg_pose[1]=ukf_msg->pose.pose.position.y;
    // msg_pose[2]=ukf_msg->pose.pose.position.z;
    // msg_quaternion=ukf_msg->pose.pose.orientation;
    // p_ukf.push_back(msg_pose);
    // p_ukf_quaternion.push_back(msg_quaternion);
    // PublishPath(ukf_path_pub,p_ukf,p_ukf_quaternion);

}

int main(int argc, char  *argv[])
{
    ros::init(argc,argv,"robot_location_path_test");
    ros::NodeHandle nh;

    odom_path_pub=nh.advertise<nav_msgs::Path>("/odom_path",10);
    odom_filter_path_pub=nh.advertise<nav_msgs::Path>("/odom_filter_path",10);
    ukf_path_pub=nh.advertise<nav_msgs::Path>("/ukf_path",10);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,"/odom",10,ros::TransportHints().tcpNoDelay());
    // message_filters::Subscriber<nav_msgs::Odometry> odom_filter_sub(nh,"/odometry/filtered_twist",10,ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<nav_msgs::Odometry> odom_filter_sub(nh,"/odometry/filtered_twist",10,ros::TransportHints().tcpNoDelay());
    // message_filters::Subscriber<nav_msgs::Odometry> ukf_sub(nh,"/odometry/ukf_filtered_twist",10,ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,nav_msgs::Odometry> syncpolicy;
    message_filters::Synchronizer<syncpolicy> sync(syncpolicy(3),odom_sub,odom_filter_sub);
    // message_filters::Synchronizer<syncpolicy> sync(syncpolicy(3),odom_sub,odom_filter_sub);
    sync.registerCallback(boost::bind(&msg_callback,_1,_2));

    ros::spin();
    return 0;
}

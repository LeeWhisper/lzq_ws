#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "instance/pid_incremental.h"
#include "instance/pid_position.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include <Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/String.h"
#include "vision/scence.h"
#include "vector"
#include <algorithm>
using namespace std;
using namespace cv;
#define K 0.1
#define PI 3.141592653
struct s_score
{
    // char name[20];
    // int age;
    // char sex;
    // char num[20];
    unordered_map <string,double> scence;

};
class laser{
    private:

        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher grid_pub;
        visualization_msgs::Marker marker;
        tf::TransformListener listener;
        ros::Subscriber sub;
        ros::Subscriber map_sub;
        ros::Subscriber scence_sub;
        vision::scence scence_data;
        string now_scence;
        // struct s_score[];

        double odd_s;
        double hit_coef = 1.22;
        double miss_coef = 0.96;
        int x_offset;
        int y_offset;
        double x,y,z,ww,zz,hh,ii,Aww,Azz,Ahh,Aii;
        double theta;
        int idx=0;
        void callback(const sensor_msgs::LaserScan::ConstPtr &laser_data);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map_data);
        void scence_callback(const vision::scence::ConstPtr & msg);
        void bresenham(Point2i p1, Point2i p2);
        cv::Point2i world2map(Point2d p);

        struct s_score
        {
            unordered_map <string,double> score;

        };
        
        std::unordered_map<std::string, int> color_set
        {
            {"defult",0},
            {"classroom",127},
            {"lab",-127}, 
            {"passage",-2} 
        };
        std::unordered_map<int, s_score> scence_score;
        int color=0;
        // float 
        
    public:
        nav_msgs::OccupancyGrid gmap;
        laser();
        bool getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& base_link);
        void get_base_pose();
        void transform_point();

};



laser::laser()
{
    sub = nh.subscribe("/scan_filtered",1,&laser::callback, this);
    map_sub = nh.subscribe("/map",1,&laser::map_callback,this);
    scence_sub = nh.subscribe("/scence",1,&laser::scence_callback,this);
    grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);
}



void laser::get_base_pose()
{
    tf::StampedTransform transform;
    try{
        //得到坐标odom和坐标base_link之间的关系
    //   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("map", "base_link",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
 
    x=transform.getOrigin().x();
    y=transform.getOrigin().y();
    z=transform.getOrigin().z();
    cout<<"x: "<<x<<endl;
    cout<<"y: "<<y<<endl;
    // cout<<"z: "<<z<<endl;
    //两种不同的表示方法，来表示getRotation
     ww=transform.getRotation()[0];
     zz=transform.getRotation()[1];
     hh=transform.getRotation()[2];
     ii=transform.getRotation()[3];

    Aii=transform.getRotation().getW();
    theta=2*acos(transform.getRotation().getW())/3.1415926*180;

}



void laser::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &map_data)
{
    gmap.header.frame_id = map_data->header.frame_id;
    gmap.header.stamp = map_data->header.stamp;
    // gmap.info.origin.position.x = 0;
    // gmap.info.origin.position.y = 0; 
    gmap.info.origin.position.x = map_data->info.origin.position.x;
    gmap.info.origin.position.y = map_data->info.origin.position.y; 
    cout<<"origin:"<<gmap.info.origin.position.x<<gmap.info.origin.position.y<<endl;
    gmap.info.origin.orientation.z = map_data->info.origin.orientation.z;
    gmap.info.origin.orientation.w = map_data->info.origin.orientation.w;
    gmap.info.resolution = map_data->info.resolution;
    gmap.info.width = map_data->info.width;
    gmap.info.height = map_data->info.height;
    gmap.data=map_data->data;
    x_offset=12.2/map_data->info.resolution;
    y_offset=15.4/map_data->info.resolution;
    // struct s_score scence_score[gmap.info.width*gmap.info.height];
}

void laser::scence_callback(const vision::scence::ConstPtr & msg)
{
    color=color_set[msg->scence];
    now_scence = msg->scence;
    cout<<"scence:"<<now_scence<<endl;
    
}

void laser::bresenham(Point2i p1, Point2i p2)
{
    // grid_map_init();
    //drawing a line from p1 to p2
    int dx = abs(p2.x - p1.x);
    int sx = (p2.x > p1.x) ? 1 : -1;
    int dy = abs(p2.y - p1.y);
    int sy = (p2.y > p1.y) ? 1 : -1;
    int err = (dx > dy ? dx : dy) / 2;
    int x1 = p1.x;
    int y1 = p1.y;
    int x2 = p2.x;
    int y2 = p2.y;
    // int x1 = p1.y;
    // int y1 = p1.x;
    // int x2 = p2.y;
    // int y2 = p2.x;
    // int8_t poi[gmap.info.width * gmap.info.height];
    // gmap.data[y1 * gmap.info.width+x1] = -127;
    while (x1 != x2 && y1 != y2)
    {
        if (x1 <0 || x2<0)
          break;
        if(!color)
            break;
        if (gmap.data[y1 * gmap.info.width+x1] == 100)
        {
            // cout<<y1 * gmap.info.height + x1<<endl;
            break;
        }
        else if (gmap.data[y1 * gmap.info.width+x1] != -1)
        {
            // cout<<y1 * gmap.info.height + x1<<endl;
            for(auto& kv:color_set)
            {
                // cout<<kv.first<<kv.second<<endl;
                if(kv.first=="defult")
                {
                    continue;
                }
                odd_s = scence_score[y1 * gmap.info.width+x1].score[now_scence] / (1-scence_score[y1 * gmap.info.width+x1].score[now_scence]);
                if(kv.first==now_scence)
                {
                    scence_score[y1 * gmap.info.width+x1].score[now_scence] = hit_coef*odd_s / (1 + hit_coef*odd_s);
                    // scence_score[y1 * gmap.info.width+x1].score[now_scence]+=1;
                    // scence_score[y1 * gmap.info.width+x1].score[now_scence] = 
                    //                                         std::min(scence_score[y1 * gmap.info.width+x1].score[now_scence],100.0);
                }
                else
                {
                    scence_score[y1 * gmap.info.width+x1].score[kv.first] = miss_coef*odd_s / (1 + miss_coef*odd_s);
                    // scence_score[y1 * gmap.info.width+x1].score[kv.first]-=0.5;
                    // scence_score[y1 * gmap.info.width+x1].score[now_scence] = 
                    //                                         std::max(scence_score[y1 * gmap.info.width+x1].score[now_scence],-100.0);
                    // cout<<"score:"<<scence_score[y1 * gmap.info.width+x1].score[kv.first]<<endl;
                }
            }

            // scence_score[y1 * gmap.info.width+x1].score[now_scence]+=1;
            if(scence_score[y1 * gmap.info.width+x1].score[now_scence] > 0.65)
            {
                gmap.data[y1 * gmap.info.width+x1] = color;
                // gmap.data[y1 * gmap.info.width+x1] = color_set[now_scence];


            }
        }
        int e2 = err;
        if (e2 > -dx) 
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dy)
        {
            err += dx;
            y1 += sy;
        }
    }
    // std::vector<int8_t> tem(poi, poi+gmap.info.width * gmap.info.height);
    // std::cout<<a.at(0)<<endl;
    // gmap.data = tem;
    grid_pub.publish(gmap);
    // cout<<"publish done"<<endl;
}

cv::Point2i laser::world2map(Point2d p)
{
    cv::Point2i ret;
    ret.x = std::ceil((p.x-gmap.info.origin.position.x)/gmap.info.resolution);
    ret.y = std::ceil((p.y-gmap.info.origin.position.y)/gmap.info.resolution);
    cout<<"map_point:"<<ret<<endl;
    return ret;
    
}


void laser::callback(const sensor_msgs::LaserScan::ConstPtr &laser_data)
{
    cout<<(laser_data->angle_max- laser_data->angle_min)/laser_data->angle_increment<<endl;
    int index=0;
    tf::TransformListener listener;
    listener.waitForTransform("/map", "/laser", ros::Time(0), ros::Duration(3.0));
    for(float i=laser_data->angle_min;i<=laser_data->angle_max;i+=laser_data->angle_increment)
    // for(float i=laser_data->angle_min/6+laser_data->angle_max;i<=laser_data->angle_max/6+laser_data->angle_max;i+=laser_data->angle_increment)
    // cout<<"size:"<<laser_data->angle_max/3/laser_data->angle_increment<<endl;
    // cout<<"index:"<<laser_data->angle_max/laser_data->angle_increment<<endl;
    // for(int i=laser_data->angle_max/laser_data->angle_increment-160;i<=laser_data->angle_max/laser_data->angle_increment+160;i++)
    {
        // cout<<"index:"<<index++<<endl;
        // cout<<"front:" <<laser_data->ranges[(i- laser_data->angle_min)/laser_data->angle_increment]<<endl;
        double range=laser_data->ranges[(i- laser_data->angle_min)/laser_data->angle_increment];
        // double range=laser_data->ranges[i];
        if(range>26)
        {
            continue;
        }
        else if(range>2)
        {
            range=2;
        }
        cout<<"range:"<<range<<endl;
        geometry_msgs::PointStamped cam_point;
        geometry_msgs::PointStamped map_point;
        cam_point.header.frame_id = "laser";
        cam_point.header.stamp = ros::Time();           
        cam_point.point.x = range * cos(i);
        cam_point.point.y = range * sin(i);
        // cam_point.point.x = res(2,0)/100;
        // cam_point.point.y = res(1,0)/100;
        cam_point.point.z = 0.0;
        try
        {   
            listener.transformPoint("map", cam_point, map_point);
            // outfile<<"px:"<<map_point.point.x<<",py:"<<map_point.point.y<<",pc:"<<temp[i].Class<<",t:"<<ros::Time::now()<<endl;
            // ROS_INFO("map: (%.2f, %.2f) ",map_point.point.x, map_point.point.y);
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR("Received an exception trying to transform a point from \"base_link\" to \"map\": %s", ex.what());
        }
        this->get_base_pose();
        
        cv::Point2i robot_now = this->world2map(cv::Point2d(x,y));
        cv::Point2i laser_now = this->world2map(cv::Point2d(map_point.point.x, map_point.point.y));
        this->bresenham(robot_now,laser_now);
        // this->bresenham(cv::Point2i(0,0),laser_now);

    }
    
    
    // cout<<"front:" <<laser_data->ranges[(laser_data->angle_max- laser_data->angle_min)/laser_data->angle_increment]

    // cout<<"front:" <<laser_data->ranges.size()<<endl;

}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "laser_front");
    laser test_laser;
    // ros::spinOnce();
    // while(test_laser.gmap.data.size())
    // {
    //     ros::spinOnce();
    // }
    ros::spin();
    
    return 0;
}

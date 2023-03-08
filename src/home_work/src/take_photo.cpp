	#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
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

using namespace std;
using namespace cv;
#define K 0.1
class photo{
    private:
        float linear_speed;
        float angle_speed;
        double theta;
        float x_now;
        float y_now;
        float x_des;
        float y_des;
        int t;
        vector<Point2f> track_vec;
        geometry_msgs::Twist twist;
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Publisher marker_pub;
        visualization_msgs::Marker marker;
        ros::Subscriber sub;
        int idx=0;
        void callback(const sensor_msgs::ImageConstPtr &colorImg);
        // float 
        
    public:
        photo();

};



photo::photo()
{
    sub = nh.subscribe("/camera/color/image_raw",1,&photo::callback, this);

}


void photo::callback(const sensor_msgs::ImageConstPtr &colorImg)
{
    cv_bridge::CvImagePtr cvImgPtr;
    try{
        cvImgPtr=cv_bridge::toCvCopy(colorImg,sensor_msgs::image_encodings::BGR8);
        
    }
    catch(cv_bridge::Exception e){
        ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
        return;
    }
    Mat color_data=cvImgPtr->image;
    imshow("color",color_data);
    double now = ros::Time::now().toSec();
    // string path = "/home/dong/semantic-planning_ws/data/roi/" + std::to_string(idx++) + ".png";
    string path = "/home/lzq/photos/lab-11-202/" + to_string(now) + ".jpg";
    imwrite(path,color_data);
    cout<<"idx:"<<idx++<<endl;
    waitKey(1000);
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "fake_photo");
    photo sim_photo;

    ros::spin();
    return 0;
}

#include "ros/ros.h"
#include "iostream"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/visualization/cloud_viewer.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_listener.h"

using namespace std;
using namespace cv;

cv_bridge::CvImagePtr rgb_cvptr;
cv_bridge::CvImagePtr depth_cvptr;
ros::Publisher pub;
Mat rgbimg, depthimg;
Eigen::Vector3f axis_z, axis_x;

const double fx = 606.436;
const double fy = 606.458;
const double cx = 316.735;
const double cy = 231.1157;
const double camera_factor = 10;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
void img_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "d435i_pointcloud");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_pointclod", 1000);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1, ros::TransportHints().tcpNoDelay());
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh,"/camera/depth/image_rect_raw",1,ros::TransportHints().tcpNoDelay());

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    ros::spin();
    return 0;
}

void img_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth)
{
    tf2_ros::Buffer buff;
    tf2_ros::TransformListener listener(buff);
    geometry_msgs::TransformStamped tfs;
    Eigen::Quaternionf q;
    Eigen::Vector3f T;
    rgb_cvptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    rgb_cvptr->image.copyTo(rgbimg);
    depth_cvptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_cvptr->image.copyTo(depthimg);

    imshow("rgb", rgbimg);
    imshow("depth", depthimg);

    PointCloud cloud;
    for (size_t v = 0; v < rgbimg.rows; v++)
    {
        for (size_t u = 0; u < rgbimg.cols; u++)
        {
            float d = depthimg.at<float>(v, u) / 1000.0;
            // if (d < 0.01 || d > 10)
            // {
            //     continue;
            // }
            PointT p;

            p.z = d;
            p.x = d * (u - cx) / fx;
            p.y = d * (v - cy) / fy;

            axis_z << p.x, p.y, p.z;

            //相机坐标系和ros坐标系的变换
            p.x = axis_z(2);
            p.y = -axis_z(0);
            p.z = -axis_z(1);

            //彩色点云对应彩色图信息
            p.b = rgbimg.at<Vec3b>(v, u)[0];
            p.g = rgbimg.at<Vec3b>(v, u)[1];
            p.r = rgbimg.at<Vec3b>(v, u)[2];

            cloud.points.push_back(p); //加入点云
        }
    }
    waitKey(1);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "camera_link";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/visualization/cloud_viewer.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "Car_Spraying/SetTwoAngle.h"

//相机内参
const double fx = 612.795654296875;
const double fy = 612.78955078125;
const double cx = 325.43609619140625;
const double cy = 236.3645782470703;
//点云计算参数
const float Pi = 3.141596;
const float Height = 0.05723;
// const float x = 0.0106;
const float x = 0.0419191 * cos(1.38749);

// const float y = 0.0175;
const float y = 0.0419191 * sin(1.38749);

const float z = 0.0125;

std::vector<pcl::PointCloud<pcl::PointXYZRGB>> Car_pointcloud; //存放四个方位的点云
std::vector<Eigen::Matrix4f> Car_matrix;
Eigen::Matrix4f translation; //当前点云的旋转平移矩阵
std_msgs::Int32 key;         //键盘按下对应的ASCII值
image_transport::Subscriber img_sub, dep_sub;
ros::Subscriber flag_sub, key_sub, servo_angle_sub;
ros::Publisher pointcloud_pub, Car_pointcloud_pub; //发布点云
// cv::Mat image;
int count = 0;
cv::Mat mask;
bool target_flag;

// void flag_get(std_msgs::Bool);
void key_flag(std_msgs::Int32);
void image_callback(const sensor_msgs::ImageConstPtr &img);
void depth_callback(const sensor_msgs::ImageConstPtr &dep);
void servo_angle_callback(const Car_Spraying::SetTwoAngle &data);
void detectHSColor(const cv::Mat &image, double minHue,
                   double maxHue, double minSat,
                   double maxSat, double minVal,
                   double maxVal, cv::Mat &mask);
Eigen::Matrix4f RotationMatrix(float theta_x, float theta_y, float theta_z);
void img_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr depth);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_segentation");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // flag_sub = nh.subscribe("/target_flag", 10, &Img::flag_get, this);
    // img_sub = it.subscribe("/camera/color/image_raw", 10, &Img::image_callback, this);
    // dep_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 10, &Img::depth_callback, this);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1, ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    key_sub = nh.subscribe("/key_board", 10, &key_flag);
    servo_angle_sub = nh.subscribe("/servo_angle", 10, &servo_angle_callback);
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcl_pointcloud", 100);
    Car_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/Car_pointcloud", 100);
    ros::spin();

    return 0;
}

/* 键盘事件回调函数 */
void key_flag(const std_msgs::Int32 get_key)
{
    std::cout << "key: " << get_key.data << std::endl;
    key.data = get_key.data;
}

/* 判断HSV阈值 */
void detectHSColor(const cv::Mat &image, double minHue,
                   double maxHue, double minSat,
                   double maxSat, double minVal,
                   double maxVal, cv::Mat &mask)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(minHue, minSat, minVal), cv::Scalar(maxHue, maxSat, maxVal), mask);
    // std::vector<cv::Mat> channels;
    // cv::split(hsv, channels);
    // cv::Mat mask1, mask2, hueMask;
    // cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
    // cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
    // if (minHue < maxHue)
    // {
    //     hueMask = mask1 & mask2;
    // }
    // else
    // {
    //     hueMask = mask1 | mask2;
    // }
    // cv::Mat satMask;
    // cv::inRange(channels[1], minSat, maxSat, satMask);
    // mask = hueMask & satMask;
    // cv::imshow("mask", mask);
}

void img_callback(const sensor_msgs::ImageConstPtr rgb, const sensor_msgs::ImageConstPtr dep)
{

    cv::Mat frame, frame_mask;
    cv_bridge::CvImagePtr color_ptr;
    color_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    frame = color_ptr->image;
    cv::imshow("frame_original", frame);

    detectHSColor(frame, 15, 40, 80, 255, 0, 255, mask); // HSV阈值判断
    frame.copyTo(frame_mask, mask);
    // cv::imshow("frame_mask", frame_mask);

    float depth;
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
    Eigen::Vector3f axis;
    cv::Mat depth_img, depth_out;
    cv_bridge::CvImagePtr depth_ptr;

    depth_ptr = cv_bridge::toCvCopy(dep, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img = depth_ptr->image;
    // depth_img.copyTo(depth_out, mask);
    depth_img.copyTo(depth_out);
    cv::imshow("depth_mask", depth_out);
    cv::waitKey(10);
    std::cout << "depth:" << depth_img.at<float>(240, 320) << std::endl;
    std::cout << "mat: " << depth_img.at<float>(0, 0) << "," << depth_img.at<float>(0, 639)
              << "," << depth_img.at<float>(479, 0) << "," << depth_img.at<float>(479, 639) << std::endl;

    for (int i = 0; i < depth_img.rows; i++)
    {
        for (int j = 0; j < depth_img.cols; j++)
        {
            depth = depth_out.at<float>(i, j) / 1000; //变换单位为m
            if (depth != 0 && depth < 1.2)            //筛选0-1.2m的深度
            {
                // pcl::PointXYZ p;
                pcl::PointXYZRGB p;
                p.z = depth;
                p.x = depth * (j - cx) / fx;
                p.y = depth * (i - cy) / fy;
                axis << p.x, p.y, p.z;

                //相机坐标系和ros坐标系的变换
                p.x = axis(2);
                p.y = -axis(0);
                p.z = -axis(1);
                p.b = frame.at<cv::Vec3b>(i, j)[0];
                p.g = frame.at<cv::Vec3b>(i, j)[1];
                p.r = frame.at<cv::Vec3b>(i, j)[2];
                // cloud.points.push_back(p); //加入点云
                cloud_rgb.points.push_back(p);
            }
        }
    }
    sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(cloud, output);
    pcl::toROSMsg(cloud_rgb, output);
    output.header.frame_id = "camera_link";
    output.header.stamp = ros::Time::now();
    pointcloud_pub.publish(output);

    /* 不知道为什么要转来转去，笔者认为这里的pointcloud_out和cloud是一样的，但是在初始化时
    更改cloud的长宽后，程序会报错，只能新建一个点云，设置长宽，从ROS格式的点云转换回来 */
    // pcl::PointCloud<pcl::PointXYZ> pointcloud_out;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_out;
    pointcloud_out.height = depth_img.rows + 1;
    pointcloud_out.width = depth_img.cols + 1;
    pcl::fromROSMsg(output, pointcloud_out);
    // pcl::io::savePCDFile<pcl::PointXYZ>("/home/lzq/lzq_ws/pointcloud/11.11/output_1.pcd", pointcloud_out);

    if (key.data == 32)
    {
        Car_pointcloud.push_back(pointcloud_out);
        Car_matrix.push_back(translation);
        std::cout << Car_pointcloud.size() << std::endl;
        key.data -= 1;
    }
    if (Car_pointcloud.size() >= 4)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB> Car_pointcloud_1[4];

        pcl::transformPointCloud(Car_pointcloud[0], Car_pointcloud_1[0], Car_matrix[0]);
        pcl::transformPointCloud(Car_pointcloud[1], Car_pointcloud_1[1], Car_matrix[1]);
        pcl::transformPointCloud(Car_pointcloud[2], Car_pointcloud_1[2], Car_matrix[2]);
        pcl::transformPointCloud(Car_pointcloud[3], Car_pointcloud_1[3], Car_matrix[3]);
        *cloud_output = Car_pointcloud_1[0] + Car_pointcloud_1[1] + Car_pointcloud_1[2] + Car_pointcloud_1[3];

        pcl::toROSMsg(*cloud_output, output);
        pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_half_1.pcd", Car_pointcloud[0]);
        pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_half_2.pcd", Car_pointcloud[1]);
        pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_half_3.pcd", Car_pointcloud[2]);
        pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_half_4.pcd", Car_pointcloud[3]);
        pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_half.pcd", *cloud_output);
        Car_pointcloud_pub.publish(output);
    }
}

/* 平移旋转矩阵函数 */
Eigen::Matrix4f RotationMatrix(float theta_x, float theta_y, float theta_z)
{
    theta_x = Pi * theta_x / 180.0;
    theta_y = Pi * theta_y / 180.0;
    theta_z = Pi * theta_z / 180.0;
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_z = Eigen::Matrix4f::Identity();
    transform_x(1, 1) = cos(theta_x);
    transform_x(1, 2) = -sin(theta_x);
    transform_x(2, 1) = sin(theta_x);
    transform_x(2, 2) = cos(theta_x);

    transform_y(0, 0) = cos(theta_y);
    transform_y(0, 2) = sin(theta_y);
    transform_y(2, 0) = -sin(theta_y);
    transform_y(2, 2) = cos(theta_y);

    transform_z(0, 0) = cos(theta_z);
    transform_z(0, 1) = -sin(theta_z);
    transform_z(1, 0) = sin(theta_z);
    transform_z(1, 1) = cos(theta_z);

    // translation = transform_x * transform_y * transform_z;
    translation = transform_z * transform_y * transform_x; // rpy角的旋转顺序为ZYX
    theta_y = -theta_y;                                    //反转方便计算，换算为xoz平面角度正方向

    //计算绕y轴旋转平移量
    float r_1 = sqrt(pow(x, 2) + pow(Height, 2));
    float theta_1 = atan(Height / x); //弧度
    float z_1 = r_1 * sin(theta_1 + theta_y);
    float x_1 = r_1 * cos(theta_1 + theta_y);
    float delta_z = -(Height - z_1);
    float delta_x_1 = x_1 - x;

    //计算绕z轴旋转平移量
    float r_2 = sqrt(pow(x_1, 2) + pow(y, 2));
    float theta_2 = atan(y / x_1);
    if (theta_2 >= 0)
        theta_2 = theta_2;
    else
        theta_2 = Pi + theta_2;
    float x_2 = r_2 * cos(theta_z + theta_2);
    float y_2 = r_2 * sin(theta_z + theta_2);
    float delta_y = y_2 - y;
    float delta_x_2 = x_2 - x_1;
    float delta_x = delta_x_1 + delta_x_2;

    translation(0, 3) = delta_x;
    translation(1, 3) = delta_y;
    translation(2, 3) = delta_z;

    return translation;
}

/* 舵机角度回调函数 */
void servo_angle_callback(const Car_Spraying::SetTwoAngle &data)
{
    // std::cout << "Real Angle0: " << data.angle0 << " "
    //           << "Angle1: " << data.angle1 << std::endl;
    translation = RotationMatrix(0, data.angle1, -data.angle0);
}

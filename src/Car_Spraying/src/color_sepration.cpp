#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
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
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/visualization/cloud_viewer.h"

// typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

cv::Point origin;          //鼠标按下的起始点
cv::Rect selection;        //定义矩形选框
bool selectObject = false; //是否选择对象
cv::Mat xyz;

class Sepration
{
private:
    const double fx = 612.795654296875;
    const double fy = 612.78955078125;
    const double cx = 325.43609619140625;
    const double cy = 236.3645782470703;
    ros::NodeHandle nh;
    image_transport::Subscriber sub_img, sub_dep;
    ros::Subscriber sub_flag, sub_key;
    ros::Publisher pub_pointcloud;
    std_msgs::Int32 key;
    double x, y, z, ww, zz, hh, ii, Aww, Azz, Ahh, Aii;
    tf::TransformListener listener;

public:
    // cv::Mat image;
    int count = 0;
    cv_bridge::CvImagePtr color_ptr, depth_ptr;
    cv::Mat mask;
    Eigen::Vector3f axis_z;
    bool target_flag;

    Sepration();
    void image_callback(const sensor_msgs::ImageConstPtr &img);
    void depth_callback(const sensor_msgs::ImageConstPtr &dep);
    void load_pointcloudfile(const sensor_msgs::ImageConstPtr &dep);
    void detectHSColor(const cv::Mat &image,
                       double minHue, double maxHue,
                       double minSat, double maxSat,
                       cv::Mat &mask);
    void flag_get(std_msgs::Bool);
    void key_flag(std_msgs::Int32);
    void get_base_pose();
};

Sepration::Sepration()
{
    image_transport::ImageTransport it(nh);
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_pointclod", 1000);
    sub_img = it.subscribe("/camera/color/image_raw", 1, &Sepration::image_callback, this);
    sub_dep = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Sepration::depth_callback, this);
    // sub_dep = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &Sepration::load_pointcloudfile, this);
    sub_flag = nh.subscribe("/target_flag", 10, &Sepration::flag_get, this);
    sub_key = nh.subscribe("/key_board", 10, &Sepration::key_flag, this);
}

void Sepration::key_flag(std_msgs::Int32 get_key)
{
    std::cout << get_key.data << std::endl;
    key.data = get_key.data;
}

void Sepration::flag_get(std_msgs::Bool flag)
{
    if (flag.data)
        target_flag = 1;
    else
        target_flag = 0;
}

void Sepration::image_callback(const sensor_msgs::ImageConstPtr &img)
{
    cv::Mat frame, frame_out;
    color_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    frame = color_ptr->image;
    // frame.copyTo(image);

    cv::imshow("frame", frame);
    detectHSColor(frame, 15, 40, 120, 255, mask);
    frame.copyTo(frame_out, mask);
    // if (target_flag)
    cv::imshow("output", frame_out);

    // cv::imwrite("/home/lzq/图片/CarDoor/output_test.png", frame);
    // cv::imwrite("/home/lzq/图片/CarDoor/output_mask_5.png", frame_out);
    cv::waitKey(10);
}

void Sepration::detectHSColor(const cv::Mat &image,
                              double minHue, double maxHue,
                              double minSat, double maxSat,
                              cv::Mat &mask)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat mask1, mask2, hueMask;
    cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
    cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
    if (minHue < maxHue)
    {
        hueMask = mask1 & mask2;
    }
    else
    {
        hueMask = mask1 | mask2;
    }
    cv::Mat satMask;
    cv::inRange(channels[1], minSat, maxSat, satMask);
    mask = hueMask & satMask;
    // cv::imshow("mask", mask);
}

void Sepration::depth_callback(const sensor_msgs::ImageConstPtr &dep)
{
    cv::Mat depth_img, depth_out;
    float depth;
    PointCloud cloud;
    depth_ptr = cv_bridge::toCvCopy(dep, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img = depth_ptr->image;
    // cv::imshow("depth", depth_img);
    // depth_img.copyTo(depth_out);
    depth_img.copyTo(depth_out, mask);
    // if (target_flag)
    cv::imshow("depth_out", depth_out);
    std::cout << "depth:" << depth_img.at<float>(320, 240) << std::endl;
    // depth_out.copyTo(xyz);
    for (int i = 0; i < depth_img.rows; i++)
    {
        for (int j = 0; j < depth_img.cols; j++)
        {
            depth = depth_out.at<float>(i, j) / 1000;
            if (depth != 0 && depth < 1.1)
            {
                // std::cout << depth << ",";
                pcl::PointXYZ p;

                p.z = depth;
                p.x = depth * (j - cx) / fx;
                p.y = depth * (i - cy) / fy;

                axis_z << p.x, p.y, p.z;

                //相机坐标系和ros坐标系的变换
                p.x = axis_z(2);
                p.y = -axis_z(0);
                p.z = -axis_z(1);
                cloud.points.push_back(p); //加入点云
                // cloud.points.erase(cloud.points.begin()+j);
            }
        }
    }
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> pointcloud_out;
    pointcloud_out.height = depth_img.rows + 1;
    pointcloud_out.width = depth_img.cols + 1;

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "camera_link";
    output.header.stamp = ros::Time::now();
    pub_pointcloud.publish(output);
    pcl::fromROSMsg(output, pointcloud_out);

    // this->get_base_pose();
    if (key.data == 32)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>> output_pointcloud;
        output_pointcloud.push_back(pointcloud_out);
        output_pointcloud.size();
        key.data -= 1;
    }

    pcl::io::savePCDFile<pcl::PointXYZ>("/home/lzq/lzq_ws/pointcloud/11.10/output_19.8_9.8.pcd", pointcloud_out);
    // cv::imwrite("/home/lzq/图片/CarDoor/depth.png", depth_img);
}

void Sepration::get_base_pose()
{
    tf::StampedTransform transform;
    try
    {
        //得到坐标odom和坐标base_link之间的关系
          listener.waitForTransform("servo_link", "camera_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("servo_link", "camera_link",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    z = transform.getOrigin().z();
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;
    cout << "z: " << z << endl;
    //两种不同的表示方法，来表示getRotation
    ww = transform.getRotation()[0];
    zz = transform.getRotation()[1];
    hh = transform.getRotation()[2];
    ii = transform.getRotation()[3];

    Aii = transform.getRotation().getW();
    // theta=2*acos(transform.getRotation().getW())/3.1415926*180;
}

void Sepration::load_pointcloudfile(const sensor_msgs::ImageConstPtr &dep)
{
    // std::cout << "123" ;
    pcl::PointCloud<pcl::PointXYZ> cloud_1, cloud_2;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lzq/lzq_ws/pointcloud/output_1.pcd", cloud_1))
    {
        PCL_ERROR("Couln't read file\n");
    }

    sensor_msgs::PointCloud2 output;
    cv::Mat depth_img, depth_out;
    float depth;
    depth_ptr = cv_bridge::toCvCopy(dep, sensor_msgs::image_encodings::TYPE_32FC1);
    depth_img = depth_ptr->image;
    depth_img.copyTo(depth_out, mask);
    cv::imshow("depth_out", depth_out);
    for (int i = 0; i < depth_img.rows; i++)
    {
        for (int j = 0; j < depth_img.cols; j++)
        {
            depth = depth_out.at<float>(i, j) / 1000;
            if (depth != 0)
            {
                // std::cout << depth << ",";
                pcl::PointXYZ p;

                p.z = depth;
                p.x = depth * (i - cx) / fx;
                p.y = depth * (j - cy) / fy;

                axis_z << p.x, p.y, p.z;

                //相机坐标系和ros坐标系的变换
                p.x = axis_z(2);
                p.y = -axis_z(1);
                p.z = -axis_z(0) - 0.1;

                cloud_2.points.push_back(p); //加入点云
            }
        }
    }

    cloud_1 += cloud_2;
    pcl::toROSMsg(cloud_1, output);
    output.header.frame_id = "camera_link";
    output.header.stamp = ros::Time::now();
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub_pointcloud.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

static void onMouse(int event, int x, int y, int, void *)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN: //鼠标左按钮按下的事件
        origin = cv::Point(x, y);
        selection = cv::Rect(x, y, 0, 0);
        selectObject = true;
        cout << "point: " << origin << endl;
        cout << "depth is: " << xyz.at<cv::Vec3f>(origin) << endl;
        // cout << xyz.at<cv::Vec3f>(origin)[0] << xyz.at<cv::Vec3f>(origin)[1] << xyz.at<cv::Vec3f>(origin)[2] << endl;

        break;
    case cv::EVENT_LBUTTONUP: //鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
            break;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Sig_id");
    Sepration sepration;
    // cv::namedWindow("depth_out", cv::WINDOW_AUTOSIZE);
    // cv::setMouseCallback("depth_out", onMouse, 0);

    ros::spin();
    return 0;
}

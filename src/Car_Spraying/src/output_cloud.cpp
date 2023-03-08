#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/pcd_io.h>
#include <cmath>

const float Pi = 3.1415926;
const float Height = 0.0565;
// const float x = 0.0106;
const float x = 0.0419191 * cos(1.38749);
// const float y = 0.0175;
const float y = 0.0419191 * sin(1.38749);
// const float z = 0.0125;
const float z = 0.0125;

std::vector<float> TranDistence(float theta_x, float theta_y, float theta_z) // 绕轴旋转位移函数
{
    // std::cout << theta_x << "," << theta_y << "," << theta_z << std::endl;
    theta_x = Pi * theta_x / 180.0;
    theta_y = -Pi * theta_y / 180.0;
    theta_z = Pi * theta_z / 180.0;
    // std::cout << theta_x << "," << theta_y << "," << theta_z << std::endl;

    std::vector<float> distence;

    // 计算绕y轴旋转平移量
    float r_1 = sqrt(pow(x, 2) + pow(Height, 2));
    float theta_1 = atan(Height / x); // 弧度
    float z_1 = r_1 * sin(theta_1 + theta_y);
    float x_1 = r_1 * cos(theta_1 + theta_y);
    float delta_z = z_1 - Height;
    float delta_x_1 = x_1 - x;
    // std::cout << delta_x_1 << std::endl;

    // 计算绕z轴旋转平移量
    float r_2 = sqrt(pow(x_1, 2) + pow(y, 2));
    // std::cout << r_2 << std::endl;
    x_1 = x;
    delta_x_1 = 0;
    float theta_2 = atan(y / x_1);
    if (theta_2 >= 0)
        theta_2 = theta_2;
    else
        theta_2 = Pi + theta_2;
    float x_2 = r_2 * cos(theta_z + theta_2);
    float y_2 = r_2 * sin(theta_z + theta_2);
    // std::cout << (theta_2+ theta_z) * 180.0 / Pi << std::endl;
    float delta_y = y_2 - y;
    float delta_x_2 = x_2 - x_1;
    float delta_x = delta_x_1 + delta_x_2;

    std::cout << "delta_x: " << delta_x << std::endl;
    std::cout << "delta_y: " << delta_y << std::endl;
    std::cout << "delta_z: " << delta_z << std::endl;

    distence.push_back(delta_x);
    distence.push_back(delta_y);
    distence.push_back(delta_z);
    return distence;
}

Eigen::Matrix4f TranslatinMatrix(float theta_x, float theta_y, float theta_z) // xyz旋转矩阵函数
{
    Eigen::Matrix4f translation;
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
    translation = transform_z * transform_y * transform_x;
    return translation;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "output_cloud");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_4(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB point;
    sensor_msgs::PointCloud2 output;

    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/test/XYZRGB_1.pcd", *cloud_1);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/test/XYZRGB_2.pcd", *cloud_2);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/test/XYZRGB_3.pcd", *cloud_3);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/test/XYZRGB_4.pcd", *cloud_4);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/11.30/sub_1.pcd", *cloud);

    for (int i = 0; i < cloud->size(); i++)
    {
        point.x = cloud->points[i].x - 0.3;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z - 0.1;
        point.r = cloud->points[i].r;
        point.g = cloud->points[i].g;
        point.b = cloud->points[i].b;
        cloud_output->points.push_back(point);
    }
    // int height, width;
    // int count_h_min, count_h_max, count_w_min, count_w_max;
    // float h_min = 0, h_max = 0, w_min = 0, w_max = 0;
    // for (int i; i <= cloud_output->size(); i++)
    // {
    //     if (h_min > cloud_output->points[i].z)
    //     {
    //         h_min = cloud_output->points[i].z;
    //         count_h_min = i;
    //     }
    //     if (h_max < cloud_output->points[i].z)
    //     {
    //         h_max = cloud_output->points[i].z;
    //         count_h_max = i;
    //     }
    //     if (w_min > cloud_output->points[i].y)
    //     {
    //         w_min = cloud_output->points[i].y;
    //         count_w_min = i;
    //     }
    //     if (w_max < cloud_output->points[i].y)
    //     {
    //         w_max = cloud_output->points[i].y;
    //         count_w_max = i;
    //     }
    // }
    // std::cout << cloud_3->points.size() << std::endl;
    // for (int i = 0; i < cloud_3->points.size(); i++)
    // {
    //     if ((cloud_3->points[i].y <= 0.001 && cloud_3->points[i].y >= -0.001) && (cloud_3->points[i].z <= 0.001 && cloud_3->points[i].z >= -0.001))
    //         std::cout << "x: " << cloud_3->points[i].x;
    // }
    // std::cout << std::endl;

    float theta_1[3] = {0, 0, 0};
    float theta_2[3] = {0, 0, 30};
    float theta_3[3] = {0, 0, 45};
    float theta_4[3] = {0, 0, -30};

    std::vector<float> DeltaDis_1, DeltaDis_2, DeltaDis_3, DeltaDis_4;
    DeltaDis_1 = TranDistence(theta_1[0], theta_1[1], -theta_1[2]);
    DeltaDis_2 = TranDistence(theta_2[0], theta_2[1], -theta_2[2]);
    DeltaDis_3 = TranDistence(theta_3[0], theta_3[1], -theta_3[2]);
    DeltaDis_4 = TranDistence(theta_4[0], theta_4[1], -theta_4[2]);

    Eigen::Matrix4f translation_1 = Eigen::Matrix4f::Identity();
    translation_1 = TranslatinMatrix(theta_1[0], (theta_1[1] * Pi) / 180, (-theta_1[2] * Pi) / 180);
    translation_1(0, 3) = DeltaDis_1[0];
    translation_1(1, 3) = DeltaDis_1[1];
    translation_1(2, 3) = DeltaDis_1[2];
    // translation_1 = translation_1.inverse().eval();
    pcl::transformPointCloud(*cloud_1, *cloud_1, translation_1);

    Eigen::Matrix4f translation_2 = Eigen::Matrix4f::Identity();
    translation_2 = TranslatinMatrix(theta_2[0], (theta_2[1] * Pi) / 180, (-theta_2[2] * Pi) / 180);
    translation_2(0, 3) = DeltaDis_2[0];
    translation_2(1, 3) = DeltaDis_2[1];
    translation_2(2, 3) = DeltaDis_2[2];
    // translation_2 = translation_2.inverse().eval();
    pcl::transformPointCloud(*cloud_2, *cloud_2, translation_2);

    Eigen::Matrix4f translation_3 = Eigen::Matrix4f::Identity();
    translation_3 = TranslatinMatrix(theta_3[0], (theta_3[1] * Pi) / 180, (-theta_3[2] * Pi) / 180);
    translation_3(0, 3) = DeltaDis_3[0];
    translation_3(1, 3) = DeltaDis_3[1];
    translation_3(2, 3) = DeltaDis_3[2];
    // translation_3 = translation_3.inverse().eval();
    pcl::transformPointCloud(*cloud_3, *cloud_3, translation_3);

    Eigen::Matrix4f translation_4 = Eigen::Matrix4f::Identity();
    translation_4 = TranslatinMatrix(theta_4[0], (theta_4[1] * Pi) / 180, (-theta_4[2] * Pi) / 180);
    translation_4(0, 3) = DeltaDis_4[0];
    translation_4(1, 3) = DeltaDis_4[1];
    translation_4(2, 3) = DeltaDis_4[2];
    // translation_4 = translation_4.inverse().eval();
    pcl::transformPointCloud(*cloud_4, *cloud_4, translation_4);

    // *cloud_output = *cloud_1 + *cloud_2 + *cloud_3 + *cloud_4;
    pcl::toROSMsg(*cloud_output, output);
    // pcl::io::savePCDFile<pcl::PointXYZ>("/home/lzq/lzq_ws/pointcloud/123.pcd", *cloud_output);

    // output.header.frame_id = "dummy";
    output.header.frame_id = "my_frame";
    output.header.stamp = ros::Time::now();
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
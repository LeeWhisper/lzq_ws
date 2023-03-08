#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "output_txtfile");
    std::ofstream outfile("/home/lzq/lzq_ws/out_3.3.txt");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/11.30/sort_ransac.pcd", *cloud);

    for (int i = 0; i < cloud->points.size(); i++)
    {
        outfile << cloud->points[i].x << "\t" << cloud->points[i].y
                << "\t" << cloud->points[i].z << std::endl;
    }

    outfile.close();
    ros::spinOnce();
    return 0;
}

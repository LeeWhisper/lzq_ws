#include <ros/ros.h>
#include <iostream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Car_Spraying/MeanFiter.h>

void BoundaryExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary, int resolution)
{
    pcl::PointXYZRGB px_min = *std::min_element(cloud->begin(), cloud->end(), [](pcl::PointXYZRGB pt1, pcl::PointXYZRGB pt2)
                                                { return pt1.x < pt2.x; });
    pcl::PointXYZRGB px_max = *std::max_element(cloud->begin(), cloud->end(), [](pcl::PointXYZRGB pt1, pcl::PointXYZRGB pt2)
                                                { return pt1.x < pt2.x; });

    float delta_x = (px_max.x - px_min.x) / resolution;
    float min_y = INT_MAX, max_y = -INT_MAX;
    std::vector<int> indexs_x(2 * resolution + 2);
    std::vector<std::pair<float, float>> minmax_x(resolution + 1, {INT_MAX, -INT_MAX});
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        int id = (cloud->points[i].x - px_min.x) / delta_x;
        if (cloud->points[i].y < minmax_x[id].first)
        {
            minmax_x[id].first = cloud->points[i].y;
            indexs_x[id] = i;
        }
        else if (cloud->points[i].y > minmax_x[id].second)
        {
            minmax_x[id].second = cloud->points[i].y;
            indexs_x[id + resolution + 1] = i;
        }
    }

    pcl::PointXYZRGB py_min = *std::min_element(cloud->begin(), cloud->end(), [](pcl::PointXYZRGB pt1, pcl::PointXYZRGB pt2)
                                                { return pt1.y < pt2.y; });
    pcl::PointXYZRGB py_max = *std::max_element(cloud->begin(), cloud->end(), [](pcl::PointXYZRGB pt1, pcl::PointXYZRGB pt2)
                                                { return pt1.y < pt2.y; });

    float delta_y = (py_max.y - py_min.y) / resolution;
    float min_x = INT_MAX, max_x = -INT_MAX;
    std::vector<int> indexs_y(2 * resolution + 2);
    std::vector<std::pair<float, float>> minmax_y(resolution + 1, {INT_MAX, -INT_MAX});
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        int id = (cloud->points[i].y - py_min.y) / delta_y;
        if (cloud->points[i].x < minmax_y[id].first)
        {
            minmax_y[id].first = cloud->points[i].x;
            indexs_y[id] = i;
        }
        else if (cloud->points[i].x > minmax_y[id].second)
        {
            minmax_y[id].second = cloud->points[i].x;
            indexs_y[id + resolution + 1] = i;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xboundary(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, indexs_x, *cloud_xboundary);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_yboundary(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, indexs_y, *cloud_yboundary);
    *cloud_boundary = *cloud_xboundary + *cloud_yboundary;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Latitude");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/11.30/HR+sub_0.005.pcd", *cloud);

    MeanFilter mf;
    mf.setInputCloud(cloud);
    // mf.setKSearch(20);
    mf.setRadiusSearch(0.03);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);
    mf.filter(*cloudOut);

    BoundaryExtraction(cloudOut, cloud_boundary, 200);

    pcl::io::savePCDFile("/home/lzq/lzq_ws/pointcloud/11.30/bouary_l.pcd", *cloud_boundary);

    //-------------------------可视化-------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_boundary, 0, 255, 0); // green
    // viewer->addPointCloud<pcl::PointXYZRGB>(cloud_boundary, single_color, "samplecloudOut cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_boundary, "samplecloudOut cloud");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    ros::spin();
    return EXIT_SUCCESS;
}

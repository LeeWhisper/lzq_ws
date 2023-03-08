#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ctime>

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/12.29/XYZRGB.pcd", *cloud);
    long startTime, endTime;
    startTime = clock();

    float min_height = 0.0, max_height = 0.0, min_width = 0.0, max_width = 0.0, width;
    int index;
    for (int i = 0; i <= cloud->points.size(); i++)
    {
        if (min_height > cloud->points[i].z)
        {
            min_height = cloud->points[i].z;
        }
        if (max_height < cloud->points[i].z)
        {
            max_height = cloud->points[i].z;
        }
    }

    // pcl::PointCloud<pcl::PointWithScale> cloudWithScale;
    // pcl::copyPointCloud(*cloud, cloudWithScale);
    // cloudWithScale.points[index].scale = 10;
    // pcl::copyPointCloud(cloudWithScale, *cloud_filtered);
    // cloud->points[index].r = 255;
    // cloud->points[index].g = 0;
    // cloud->points[index].b = 0;

    // std::cout << "min_height: " << min_height << std::endl;
    // std::cout << "min_width: " << min_width << std::endl;

    pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(min_height, min_height + 0.557);
    pass_z.setFilterLimitsNegative(true); // 为true则滤除范围内点云
    pass_z.filter(*cloud_filtered);

    pass_z.setInputCloud(cloud_filtered);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(max_height - 0.205, max_height);
    pass_z.setFilterLimitsNegative(true); // 为true则滤除范围内点云
    pass_z.filter(*cloud_filtered);
    for (int i = 0; i <= cloud_filtered->points.size(); i++)
    {
        if (max_width < cloud_filtered->points[i].y)
        {
            max_width = cloud_filtered->points[i].y;
        }
        if (min_width > cloud_filtered->points[i].y)
        {
            min_width = cloud_filtered->points[i].y;
        }
    }

    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(cloud_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(max_width - 0.527, max_width);
    pass_y.setFilterLimitsNegative(true); // 为true则滤除范围内点云
    pass_y.filter(*cloud_filtered);

    pass_y.setInputCloud(cloud_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(min_width, min_width + 0.51);
    pass_y.setFilterLimitsNegative(true); // 为true则滤除范围内点云
    pass_y.filter(*cloud_filtered);

    endTime = clock();
    std::cout << "run time: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

    min_width = 0.0;
    max_width = 0.0;
    for (int i = 0; i <= cloud_filtered->points.size(); i++)
    {
        if (max_width < cloud_filtered->points[i].y)
        {
            max_width = cloud_filtered->points[i].y;
        }
        if (min_width > cloud_filtered->points[i].y)
        {
            min_width = cloud_filtered->points[i].y;
        }
    }
    width = max_width - min_width;
    std::cout << "width: " << width << std::endl;
    pcl::PCDWriter writer;
    writer.write("/home/lzq/lzq_ws/pointcloud/12.29/CloudWithHeightRemoval.pcd", *cloud_filtered, true);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("filter"));
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1); // 设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
    viewer->setBackgroundColor(0, 0, 0, v1);        // 设置背景颜色，0-1，默认黑色（0，0，0）
    viewer->addText("after_filtered", 10, 10, "v1_text", v1);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "after_filtered_cloud", v1);
    viewer->addCoordinateSystem(0.5);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}

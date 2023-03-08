#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZRGB> restrain(pcl::PointCloud<pcl::PointXYZRGB> pc_in, pcl::PointCloud<pcl::PointXYZRGB> pc_border);
float min_y = 0.0, max_y = 0.0;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "color_restrain");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_border(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> pc_border;
    pcl::PointCloud<pcl::PointXYZRGB> pc_in;

    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB.pcd", *cloud_in);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/3.1/border_SOR.pcd", *cloud_border);

    float max_z;
    int count;
    for (int i = 0; i < cloud_border->points.size(); i++)
    {
        if (max_z < cloud_border->points[i].z)
            max_z = cloud_border->points[i].z;
        if (max_y < cloud_border->points[i].y)
            max_y = cloud_border->points[i].y;
        if (min_y > cloud_border->points[i].y)
            min_y = cloud_border->points[i].y;
    }

    std::cout << "max_z: " << max_z << ", min_y: " << min_y << ", max_y: " << max_y << std::endl;

    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        if ((max_z - 0.025 < cloud_in->points[i].z) && cloud_in->points[i].z <= max_z)
            pc_in.push_back(cloud_in->points[i]);
    }
    for (int i = 0; i < cloud_border->points.size(); i++)
    {
        if ((max_z - 0.025 < cloud_border->points[i].z) && cloud_border->points[i].z <= max_z)
        {
            pc_border.push_back(cloud_border->points[i]);
            count++;
        }
    }

    pc_in = restrain(pc_in, pc_border);
    for (int i = 0; i < pc_in.size(); i++)
    {
        cloud_out->points.push_back(pc_in[i]);
    }
    pc_in.clear();
    pc_border.clear();
    max_z -= 0.025;

    while (count < cloud_border->points.size())
    {
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            if ((max_z - 0.05 < cloud_in->points[i].z) && cloud_in->points[i].z <= max_z)
                pc_in.push_back(cloud_in->points[i]);
        }
        for (int i = 0; i < cloud_border->points.size(); i++)
        {
            if ((max_z - 0.05 < cloud_border->points[i].z) && cloud_border->points[i].z <= max_z)
            {
                pc_border.push_back(cloud_border->points[i]);
                count++;
            }
        }
        pc_in = restrain(pc_in, pc_border);
        for (int i = 0; i < pc_in.size(); i++)
        {
            cloud_out->points.push_back(pc_in[i]);
        }
        pc_in.clear();
        pc_border.clear();
        max_z -= 0.05;
        // std::cout << count << std::endl;
    }
    cloud_out->height = 1;
    cloud_out->width = cloud_out->points.size();
    cloud_out->resize(cloud_out->height * cloud_out->width);

    pcl::io::savePCDFile<pcl::PointXYZRGB>("lzq_ws/pointcloud/3.1/XYZRGB_Re.pcd", *cloud_out);
    //-------------------------可视化-------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_out, "cloudOut cloud");
    viewer->setBackgroundColor(0.15, 0.15, 0.15);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    ros::spin();
    return 0;
}

pcl::PointCloud<pcl::PointXYZRGB> restrain(const pcl::PointCloud<pcl::PointXYZRGB> pc_in, const pcl::PointCloud<pcl::PointXYZRGB> pc_border)
{
    // std::cout << "123 " << std::endl;

    float min_y_res = 0.0, max_y_res = 0.0;
    pcl::PointCloud<pcl::PointXYZRGB> out;

    for (int i = 0; i < pc_border.size(); i++)
    {
        if (min_y_res > pc_border[i].y)
            min_y_res = pc_border[i].y;
        if (max_y_res < pc_border[i].y)
            max_y_res = pc_border[i].y;
    }
    // 微元边界的最值不足1/2，则直接赋值为边界最值
    if (max_y_res < (min_y + (max_y - min_y) / 2))
        max_y_res = max_y;
    if (min_y_res > (min_y + (max_y - min_y) / 2))
        min_y_res = min_y;

    std::cout << "min_y_res: " << min_y_res << ", max_y_res: " << max_y_res << std::endl;
    for (int i = 0; i < pc_in.size(); i++)
    {
        if ((min_y_res < pc_in[i].y) && (pc_in[i].y < max_y_res))
            out.push_back(pc_in[i]);
    }
    //------------------------------排序----------------------------
    std::sort(out.begin(), out.end(),
              [](pcl::PointXYZRGB a, pcl::PointXYZRGB b)
              { return a.y > b.y; });

    return out;
}
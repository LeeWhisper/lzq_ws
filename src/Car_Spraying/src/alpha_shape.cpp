// #include <ros/ros.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>

// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "tmp_pointcloud");
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//     // pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/11.30/HR+sub+De_1.pcd", *cloud);
//     pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/11.30/XYZRGB.pcd", *cloud);

//     for (int i = 0; i < cloud->points.size(); i++)
//     {
//         cloud->points[i].x = 0;
//     }
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

//     //-------------------------可视化-------------------------
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0); // green

//     viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "samplecloudOut cloud");
//     viewer->setBackgroundColor(0.3, 0.3, 0.3);

//     while (!viewer->wasStopped())
//     {
//         viewer->spinOnce(100);
//         // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//     }

//     ros::spin();
//     return 0;
// }
// ashape.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <pcl/common/common.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

double get2DDist(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2)
{
    double dist = std::pow(((p1.z - p2.z) * (p1.z - p2.z) + (p1.y - p2.y) * (p1.y - p2.y)), 0.5);
    return dist;
}
int main()
{
    std::string file_name("/home/lzq/lzq_ws/pointcloud/11.30/sub_0.05.pcd");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_initial(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_name, *cloud_in);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_name, *cloud_initial);
    // 投影到二维平面
    for (auto &point : *cloud_in)
    {
        point.x = 0.0;
    }
    // pcl::io::savePCDFile<pcl::PointXYZRGB>("二维平面.pcd", *cloud_in);
    std::vector<int> boundary_bool(cloud_in->size(), 0);
    double r = 0.05;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud_in);
    std::vector<int> indices(0, 0);
    std::vector<float> dist(0, 0.0);
    for (size_t i = 0; i < cloud_in->size(); i++)
    {
        // std::cout << "i:" << i << "/" << cloud_in->size()-1 << std::endl;
        pcl::PointXYZRGB p = cloud_in->points[i];
        kdtree.radiusSearch(cloud_in->points[i], 2 * r, indices, dist);
        // std::cout << "indices.size():" << indices.size() << std::endl;

        // indices[0]对应的点云即为查询点本身
        for (size_t j = 1; j < indices.size(); ++j)
        {
            pcl::PointXYZRGB p1 = cloud_in->points[indices[j]];
            // std::cout << p.x << ", " << p.y << ", " << p.z << std::endl;
            // std::cout << p1.x << ", " << p1.y << ", " << p1.z << std::endl;
            // std::cout << "indices:" << indices[j] << std::endl;
            double s_2 = std::pow((p.z - p1.z), 2) + std::pow((p.y - p1.y), 2);
            double h = std::pow((r * r / s_2 - 0.25), 0.5);
            double z2 = p.z + 0.5 * (p1.z - p.z) - h * (p1.y - p.y);
            double y2 = p.y + 0.5 * (p1.y - p.y) - h * (p.z - p1.z);
            double z3 = p.z + 0.5 * (p1.z - p.z) + h * (p1.y - p.y);
            double y3 = p.y + 0.5 * (p1.y - p.y) + h * (p.z - p1.z);
            // std::cout << "s_2: " << s_2 << std::endl;
            // std::cout << "h: " << h << std::endl;
            // std::cout << "z2: " << z2 << std::endl;

            /* 不能写成 pcl::PointXYZRGB p2(0.0, y2, z2)，因为初始化点，只能对RGB值进行定义
            但是pcl::PointXYZ p4(1.0, 2.0, 3.0)，则可以对XYZ赋值 */

            pcl::PointXYZRGB p2, p3;
            p2.x = 0.0;
            p2.y = y2;
            p2.z = z2;
            p3.x = 0.0;
            p3.y = y3;
            p3.z = z3;

            // std::cout << p2.x << ", " << p2.y << ", " << p2.z << std::endl;
            // std::cout << p3.x << ", " << p3.y << ", " << p3.z << std::endl;

            // 计算邻域内除了p1之外的点到p2,p3的距离
            std::vector<double> distp2, distp3;
            std::vector<int> distp2_bool(0, 0), distp3_bool(0, 0);
            int count = 0;
            for (size_t k = 1; k < indices.size(); ++k)
            {
                pcl::PointXYZRGB p_other = cloud_in->points[indices[k]];
                if (k != j)
                {
                    ++count;
                    double distance_p2 = get2DDist(p_other, p2);
                    double distance_p3 = get2DDist(p_other, p3);
                    // std::cout << p.x << ", " << p.y << ", " << p.z << std::endl;
                    // std::cout << p2.x << ", " << p2.y << ", " << p2.z << std::endl;
                    // std::cout << p3.x << ", " << p3.y << ", " << p3.z << std::endl;
                    // std::cout << p_other.x << ", " << p_other.y << ", " << p_other.z << std::endl;

                    // 比较距离与r的大小
                    if (distance_p2 > r)
                    {
                        distp2_bool.push_back(1);
                    }
                    if (distance_p3 > r)
                    {
                        distp3_bool.push_back(1);
                    }
                }
            }
            // 如果邻域内所有点到p2，或者p3的距离均大于r，则有distp2_bool.size()==count
            // 则表明p是边界点,退出循环，不用再计算邻域内点距离了
            if (count == distp2_bool.size() || count == distp3_bool.size())
            {
                // std::cout << "yes" << std::endl;
                boundary_bool[i] = 1;
                break;
            }
        }
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t it = 0; it < boundary_bool.size(); ++it)
    {
        if (boundary_bool[it] == 1)
        {
            boundary_cloud->points.push_back(cloud_initial->points[it]);
        }
    }
    boundary_cloud->height = boundary_cloud->points.size();
    boundary_cloud->width = 1;
    boundary_cloud->resize(boundary_cloud->height * boundary_cloud->width);
    if (boundary_cloud->size())
    {
        // pcl::io::savePCDFile<pcl::PointXYZRGB>("边界点云.pcd", boundary_cloud);
        //-------------------------可视化-------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->addPointCloud<pcl::PointXYZRGB>(boundary_cloud, "samplecloudOut cloud");
        viewer->setBackgroundColor(0, 0, 0);

        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
}

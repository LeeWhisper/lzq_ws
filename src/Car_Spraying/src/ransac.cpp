#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// 函数：RANSAC(随机采样一致性算法),基于pcl实现，与原有的pcl平面分割算法的区别在于：在创建平面的时候考虑到了点云的颜色信息，输入对象为具有颜色信息的点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Ransac_Plane_With_Color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc /* 输入点云 */,
                                                               int iter /* 迭代次数 */,
                                                               double dis_thresh /* 距离容忍度 */,
                                                               int color_thresh /* 颜色容忍度 */)
{
    // 最终要返回的结果
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr res(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 输入点云的大小
    int Size_pc = pc->points.size();
    if (Size_pc <= 10) // 点云数量不够不能进行拟合
    {
        return pc;
    }

    // 空间直线拟合

    for (int i = 0; i < Size_pc; i++)
    {
        int p1, p2;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
        p1 = i;
        for (int i = 0; i < Size_pc; i++)
        {
            p2 = i;
            if (p1 == p2)
                continue;
            // std::cout << "p1: " << p1 << ", p2: " << p2 << std::endl;
            // 将两个点存入临时变量点云Tmp中
            Tmp->points.push_back(pc->points[p1]);
            Tmp->points.push_back(pc->points[p2]);
            // 使用两个点，拟合一条空间直线
            //  # M = x2 - x1;
            //  # N = y2 - y1;
            //  # P = z2 - z1;
            //  空间直线方程表达式：(x - x1) / M = (y - y1) / N = (z - z1) / P;
            double M = Tmp->points[p2].x - Tmp->points[p1].x;
            double N = Tmp->points[p2].y - Tmp->points[p1].y;
            double P = Tmp->points[p2].z - Tmp->points[p1].z;
            // 遍历整个输入点云，计算每个点云与直线的距离，并将内点存入临时变量点云中
            for (int i = 0; i < pc->points.size(); i++)
            {
                if (i != p1 && i != p2)
                {
                    double t = (M * (pc->points[i].x - Tmp->points[p1].x) + N * (pc->points[i].y - Tmp->points[p1].y) + P * (pc->points[i].z - Tmp->points[p1].z)) / (M * M + N * N + P * P);
                    double x_c = M * t + Tmp->points[p1].x;
                    double y_c = N * t + Tmp->points[p1].y;
                    double z_c = P * t + Tmp->points[p1].z;
                    double dis = sqrt((pc->points[i].x - x_c) * (pc->points[i].x - x_c) + (pc->points[i].y - y_c) * (pc->points[i].y - y_c) + (pc->points[i].z - z_c) * (pc->points[i].z - z_c));
                    if (dis < dis_thresh)
                        Tmp->points.push_back(pc->points[i]);
                }
            }
            // 比较临时变量点云与res的大小，选取点云数多的点云，并释放new申请的空间
            if (Tmp->points.size() > res->points.size())
                Tmp->swap(*res);
            Tmp->clear();
        }
    }
    // for (int i = 0; i < iter; i++)
    // {
    //     std::cout << "iter:" << i << std::endl;
    //     // 临时参数初始化
    //     int p1, p2;
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    //     // 随机选取两个点，需满足不是相同点
    //     while (true)
    //     {
    //         // 随机选取两个点
    //         p1 = random() % Size_pc;
    //         p2 = random() % Size_pc;
    //         // 是否存在相同的点
    //         if (p1 == p2)
    //             continue;
    //         break;
    //     }
    //     // 将两个点存入临时变量点云Tmp中
    //     Tmp->points.push_back(pc->points[p1]);
    //     Tmp->points.push_back(pc->points[p2]);
    //     // 使用两个点，拟合一条空间直线
    //     //  # M = x2 - x1;
    //     //  # N = y2 - y1;
    //     //  # P = z2 - z1;
    //     //  空间直线方程表达式：(x - x1) / M = (y - y1) / N = (z - z1) / P;
    //     double M = Tmp->points[p2].x - Tmp->points[p1].x;
    //     double N = Tmp->points[p2].y - Tmp->points[p1].y;
    //     double P = Tmp->points[p2].z - Tmp->points[p1].z;
    //     // 遍历整个输入点云，计算每个点云与直线的距离，并将内点存入临时变量点云中
    //     for (int i = 0; i < pc->points.size(); i++)
    //     {
    //         if (i != p1 && i != p2)
    //         {
    //             double t = (M * (pc->points[i].x - Tmp->points[p1].x) + N * (pc->points[i].y - Tmp->points[p1].y) + P * (pc->points[i].z - Tmp->points[p1].z)) / (M * M + N * N + P * P);
    //             double x_c = M * t + Tmp->points[p1].x;
    //             double y_c = N * t + Tmp->points[p1].y;
    //             double z_c = P * t + Tmp->points[p1].z;
    //             double dis = sqrt((pc->points[i].x - x_c) * (pc->points[i].x - x_c) + (pc->points[i].y - y_c) * (pc->points[i].y - y_c) + (pc->points[i].z - z_c) * (pc->points[i].z - z_c));
    //             if (dis < dis_thresh)
    //                 Tmp->points.push_back(pc->points[i]);
    //         }
    //     }
    //     // 比较临时变量点云与res的大小，选取点云数多的点云，并释放new申请的空间
    //     if (Tmp->points.size() > res->points.size())
    //         Tmp->swap(*res);
    //     Tmp->clear();
    // }

    // 平面拟合
    // for (int i = 0; i < iter; i++)
    // {
    //     // std::cout << "iter:" << i << std::endl;
    //     // 临时参数初始化
    //     int p1, p2, p3;
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    //     // 随机选取三个点，需满足：1、三点不共线；2、三点颜色近似
    //     while (true)
    //     {
    //         // 随机选取三个点
    //         p1 = random() % Size_pc;
    //         p2 = random() % Size_pc;
    //         p3 = random() % Size_pc;
    //         // 是否存在相同的点
    //         if (p1 == p2 || p1 == p3 || p2 == p3)
    //             continue;
    //         // 判断三点是否共线
    //         if ((pc->points[p1].x - pc->points[p2].x) * (pc->points[p1].y - pc->points[p3].y) == (pc->points[p1].x - pc->points[p3].x) * (pc->points[p1].y - pc->points[p2].y))
    //             continue;
    //         // 判断三点的颜色是否近似
    //         // if (abs(pc->points[p1].r - pc->points[p2].r) > color_thresh || abs(pc->points[p1].g - pc->points[p2].g) > color_thresh || abs(pc->points[p1].b - pc->points[p2].b) > color_thresh ||
    //         //     abs(pc->points[p1].r - pc->points[p3].r) > color_thresh || abs(pc->points[p1].g - pc->points[p3].g) > color_thresh || abs(pc->points[p1].b - pc->points[p3].b) > color_thresh ||
    //         //     abs(pc->points[p3].r - pc->points[p2].r) > color_thresh || abs(pc->points[p3].g - pc->points[p2].g) > color_thresh || abs(pc->points[p3].b - pc->points[p2].b) > color_thresh)
    //         //     continue;
    //         // 找到合适的三个点
    //         break;
    //     }
    //     // 将三个点存入临时变量点云Tmp中
    //     Tmp->points.push_back(pc->points[p1]);
    //     Tmp->points.push_back(pc->points[p2]);
    //     Tmp->points.push_back(pc->points[p3]);
    //     // 使用三个点，拟合一个平面
    //     //  # A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
    //     //  # B = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
    //     //  # C = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
    //     //  平面方程表达式：A*x + B*y + C*z + D = 0
    //     double A = (pc->points[p2].y - pc->points[p1].y) * (pc->points[p3].z - pc->points[p1].z) - (pc->points[p2].z - pc->points[p1].z) * (pc->points[p3].y - pc->points[p1].y);
    //     double B = (pc->points[p3].x - pc->points[p1].x) * (pc->points[p2].z - pc->points[p1].z) - (pc->points[p3].z - pc->points[p1].z) * (pc->points[p2].x - pc->points[p1].x);
    //     double C = (pc->points[p3].y - pc->points[p1].y) * (pc->points[p2].x - pc->points[p1].x) - (pc->points[p3].x - pc->points[p1].x) * (pc->points[p2].y - pc->points[p1].y);
    //     double D = -A * pc->points[p1].x - B * pc->points[p1].y - C * pc->points[p1].z;
    //     // 遍历整个输入点云，计算每个点云与平面的距离，并将内点存入临时变量点云中
    //     double root = sqrt(A * A + B * B + C * C);
    //     for (int i = 0; i < pc->points.size(); i++)
    //     {
    //         if (i != p1 && i != p2 && i != p3)
    //         {
    //             double dis = abs(A * pc->points[i].x + B * pc->points[i].y + C * pc->points[i].z + D) / root;
    //             if (dis < dis_thresh)
    //                 Tmp->points.push_back(pc->points[i]);
    //         }
    //     }
    //     // 比较临时变量点云与res的大小，选取点云数多的点云，并释放new申请的空间
    //     if (Tmp->points.size() > res->points.size())
    //         Tmp->swap(*res);
    //     Tmp->clear();
    // }
    // 返回最终结果
    return res;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ransac");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_UniSmp.pcd", *cloud);
    std::cout << "\t\t<读入点云信息>\n"
              << *cloud << std::endl;
    float max_z = 0, count = 0;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (max_z <= cloud->points[i].z)
        {
            max_z = cloud->points[i].z;
        }
    }

    for (int i = 0; i < cloud->points.size(); i++)
    {
        if ((max_z - 0.01 < cloud->points[i].z) && cloud->points[i].z <= max_z)
        {
            cloud_temp->points.push_back(cloud->points[i]);
            count++;
        }
    }
    // std::cout << count << std::endl;
    cloud_temp = Ransac_Plane_With_Color(cloud_temp, 10000, 0.015, 0);

    for (int i = 0; i < cloud_temp->points.size(); i++)
    {
        cloud_out->points.push_back(cloud_temp->points[i]);
    }
    cloud_temp->clear();
    max_z -= 0.01;

    while (count < cloud->points.size())
    {
        for (int i = 0; i < cloud->points.size(); i++)
        {
            if ((max_z - 0.05 < cloud->points[i].z) && cloud->points[i].z <= max_z)
            {
                cloud_temp->points.push_back(cloud->points[i]);
                count++;
            }
        }
        // std::cout << count << std::endl;
        cloud_temp = Ransac_Plane_With_Color(cloud_temp, 10000, 0.02, 0);

        for (int i = 0; i < cloud_temp->points.size(); i++)
        {
            cloud_out->points.push_back(cloud_temp->points[i]);
        }
        cloud_temp->clear();
        max_z -= 0.05;
    }
    pcl::PCDWriter writer;
    writer.write("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_UniSmp_ransac.pcd", *cloud_out, true);
    std::cout << "\t\t<保存点云信息>\n"
              << *cloud_out << std::endl;
    //-------------------------可视化-------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_out, 0, 255, 0); // green

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_out, single_color, "samplecloudOut cloud");
    viewer->setBackgroundColor(0, 0, 0);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    ros::spin();
    return 0;
}

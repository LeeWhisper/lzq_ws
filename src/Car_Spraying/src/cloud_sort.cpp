#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void bubbleSort(int arr[], int n)
{
    for (int i = 0; i < n; i++)
    {
        // 比较两个相邻的元素
        for (int j = 0; j < n - i - 1; j++)
        {
            if (arr[j] < arr[j + 1])
            {
                int t = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = t;
            }
        }
    }
}
int flag = 1;

std::vector<pcl::PointXYZRGB> Sort(std::vector<pcl::PointXYZRGB> vec)
{
    int n = vec.size();
    std::vector<pcl::PointXYZRGB> vec_out;
    pcl::PointXYZRGB point;

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n - i - 1; j++)
        {
            if (flag == 1)
            {
                if (vec[j].y > vec[j + 1].y)
                {
                    point = vec[j];
                    vec[j] = vec[j + 1];
                    vec[j + 1] = point;
                }
            }
            else
            {
                if (vec[j].y < vec[j + 1].y)
                {
                    point = vec[j];
                    vec[j] = vec[j + 1];
                    vec[j + 1] = point;
                }
            }
        }
    }
    flag = -flag;

    return vec;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cloud_sort");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_UniSmp_ransac.pcd", *cloud);
    std::vector<pcl::PointXYZRGB> vec;
    float max_z = 0, count = 0;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (max_z <= cloud->points[i].z)
        {
            max_z = cloud->points[i].z;
        }
    }\
    // max_z -= 0.025;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if ((max_z - 0.01 < cloud->points[i].z) && cloud->points[i].z <= max_z)
        {
            vec.push_back(cloud->points[i]);
            count++;
        }
    }
    vec = Sort(vec);
    for (int i = 0; i < vec.size(); i++)
    {
        cloud_out->points.push_back(vec[i]);
    }
    vec.clear();
    max_z -= 0.01;

    while (count < cloud->points.size())
    {
        for (int i = 0; i < cloud->points.size(); i++)
        {
            if ((max_z - 0.05 < cloud->points[i].z) && cloud->points[i].z <= max_z)
            {
                vec.push_back(cloud->points[i]);
                count++;
            }
        }
        vec = Sort(vec);
        for (int i = 0; i < vec.size(); i++)
        {
            cloud_out->points.push_back(vec[i]);
        }
        vec.clear();
        max_z -= 0.05;
        // std::cout << count << std::endl;
    }
    // pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/11.30/sort.pcd", *cloud_out);
    std::cout << "点云顺序转换完毕！" << std::endl;

    pcl::PCDWriter writer;
    writer.write("/home/lzq/lzq_ws/pointcloud/3.1/sort.pcd", *cloud_out, true);

    //-------------------------可视化-------------------------
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_out, 0, 255, 0); // green

    // viewer->addPointCloud<pcl::PointXYZRGB>(cloud_out, single_color, "samplecloudOut cloud");
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    //     // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }

    ros::spinOnce();
    return 0;
}

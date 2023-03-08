#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Car_Spraying/MeanFiter.h>

bool color(pcl::PointXYZRGB point)
{
    bool flag = 0;
    int R, G, B;
    int min_R, max_R, min_G, max_G, min_B, max_B;

    R = (int)point.r;
    G = (int)point.g;
    B = (int)point.b;
    // min_R = 100;
    // max_R = 175;
    // min_G = 50;
    // max_G = 110;
    // min_B = 0;
    // max_B = 100;

    min_R = 100;
    max_R = 255;
    min_G = 150;
    max_G = 255;
    min_B = 0;
    max_B = 210;

    if (R >= min_R && R <= max_R && G >= min_G && G <= max_G && B >= min_B && B <= max_B)
    {
        flag = 1;
    }

    return flag;
}

pcl::PointCloud<pcl::PointXYZRGB> vertex(pcl::PointCloud<pcl::PointXYZRGB> cloud, float z_max, float z_min, float y_max, float y_min)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    for (int i = 0; i < cloud.points.size(); i++)
    {
        if (cloud.points[i].z <= z_max && cloud.points[i].z >= z_min && cloud.points[i].y <= y_max && cloud.points[i].y >= y_min)
        {
            cloud_filtered.push_back(cloud.points[i]);
        }
    }
    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB> delet_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool setNegative)
{
    std::vector<int> total_index;
    for (size_t i = 0; i < cloud->size(); i++)
    {
        if (color(cloud->points[i]))
        {
            total_index.push_back(i);
        }
    }
    sort(total_index.begin(), total_index.end()); // 将索引进行排序
    //-------------------根据索引删除重复的点-------------------
    pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
    outliners->indices.resize(total_index.size());
    for (size_t i = 0; i < total_index.size(); i++)
    {
        outliners->indices[i] = total_index[i];
    }
    //-------------------提取删除重复点之后的点云--------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(outliners);
    extract.setNegative(setNegative); // 设置为true则表示保存索引之外的点
    extract.filter(*cloud_filtered);

    return (*cloud_filtered);
}

//-------------------体素滤波--------------
pcl::PointCloud<pcl::PointXYZRGB> voxelGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float LeafSize)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "->正在体素下采样..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;          // 创建滤波器对象
    vg.setInputCloud(cloud);                      // 设置待滤波点云
    vg.setLeafSize(LeafSize, LeafSize, LeafSize); // 设置体素大小，单位是m
    vg.filter(*cloud_filtered);                   // 执行滤波，保存滤波结果于cloud_filtered
    return (*cloud_filtered);
}

//-------------------统计滤波--------------
pcl::PointCloud<pcl::PointXYZRGB> statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float Meank = 50.0, float StddevMulThresh = 1.0)
{
    std::cout << "->正在进行统计滤波..." << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; // 创建滤波器对象
    sor.setInputCloud(cloud);                             // 设置待滤波点云
    sor.setMeanK(Meank);                                  // 设置查询点近邻点的个数
    sor.setStddevMulThresh(StddevMulThresh);              // 设置标准差乘数，来计算是否为离群点的阈值
    sor.setNegative(false);                               // 默认false，保存内点；true，保存滤掉的离群点
    sor.filter(*cloud_filtered);                          // 执行滤波，保存滤波结果于cloud_filtered
    return (*cloud_filtered);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vertex_bind");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/11.30/XYZRGB.pcd", *cloud);
    float z_max = 0, z_min = 0, y_max = 0, y_min = 0;

    //-------------------提取删除重复点之后的点云--------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_filtered = delet_color(cloud, false);
    pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/11.30/vertex.pcd", *cloud_filtered);

    //-------------------体素滤波--------------
    std::cout << "->正在体素下采样..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vg; // 创建滤波器对象
    vg.setInputCloud(cloud_filtered);    // 设置待滤波点云
    vg.setLeafSize(0.1f, 0.1f, 0.1f);    // 设置体素大小，单位是m
    vg.filter(*cloud_filtered);          // 执行滤波，保存滤波结果于cloud_filtered

    //-------------------统计滤波--------------
    std::cout << "->正在进行统计滤波..." << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; // 创建滤波器对象
    sor.setInputCloud(cloud_filtered);                    // 设置待滤波点云
    sor.setMeanK(25);                                     // 设置查询点近邻点的个数
    sor.setStddevMulThresh(0.75);                         // 设置标准差乘数，来计算是否为离群点的阈值
    sor.setNegative(false);                               // 默认false，保存内点；true，保存滤掉的离群点
    sor.filter(*cloud_filtered);                          // 执行滤波，保存滤波结果于cloud_filtered

    //-----------------保存下采样点云-----------
    for (int i = 0; i < cloud_filtered->points.size(); i++)
    {
        if (cloud_filtered->points[i].z >= z_max)
        {
            z_max = cloud_filtered->points[i].z;
        }
        if (cloud_filtered->points[i].z <= z_min)
        {
            z_min = cloud_filtered->points[i].z;
        }
        if (cloud_filtered->points[i].y >= y_max)
        {
            y_max = cloud_filtered->points[i].y;
        }
        if (cloud_filtered->points[i].y <= y_min)
        {
            y_min = cloud_filtered->points[i].y;
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vertex(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_vertex = vertex(*cloud, z_max, z_min, y_max, y_min);
    *cloud_vertex = delet_color(cloud_vertex, true);
    *cloud_vertex = statisticalOutlierRemoval(cloud_vertex, 20, 1);
    *cloud_vertex = voxelGrid(cloud_vertex, 0.01);
    // pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/11.30/temporary.pcd", *cloud_vertex);
    // *cloud_vertex = statisticalOutlierRemoval(cloud_vertex, 5, 1);
    MeanFilter mf;
    mf.setInputCloud(cloud_vertex);
    // mf.setKSearch(20);
    mf.setRadiusSearch(0.03);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);
    mf.filter(*cloudOut);
    *cloud_vertex = voxelGrid(cloudOut, 0.05);

    *cloud_vertex = statisticalOutlierRemoval(cloud_vertex, 5, 1);
    *cloud_vertex = statisticalOutlierRemoval(cloud_vertex, 5, 1.1);
    *cloud_vertex = statisticalOutlierRemoval(cloud_vertex, 5, 1.2);

    // std::cout << cloud_vertex->points.size() << std::endl;
    // mf.setInputCloud(cloud_vertex);
    // // mf.setKSearch(20);
    // mf.setRadiusSearch(0.03);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut_1(new pcl::PointCloud<pcl::PointXYZRGB>);

    // mf.filter(*cloudOut_1);
    // *cloud_vertex = voxelGrid(cloudOut_1, 0.05);

    pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/11.30/vertex_1.pcd", *cloud_vertex);

    //-------------------------可视化-------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_vertex, 0, 255, 0); // green

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_vertex, single_color, "samplecloudOut cloud");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    ros::spin();
    return 0;
}

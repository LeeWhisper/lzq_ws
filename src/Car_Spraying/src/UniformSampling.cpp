#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h> // 均匀采样
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

void visualize_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &filter_cloud)
{
    //---------显示点云-----------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("显示点云"));

    int v1(0), v2(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("point clouds", 10, 10, "v1_text", v1);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZRGB>(filter_cloud, "cloud_filtered", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    // viewer->addCoordinateSystem(1.0);
    // viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De.pcd", *cloud) != 0)
    {
        return -1;
    }

    cout << "原始点云个数：" << cloud->points.size() << endl;
    // ----------------创建均匀采样对象-------------------------
    pcl::UniformSampling<pcl::PointXYZRGB> US;
    US.setInputCloud(cloud);
    US.setRadiusSearch(0.025f); // 设置滤波时创建球体的半径
    US.filter(*cloud_filtered);
    cout << "均匀采样之后点云的个数：" << cloud_filtered->points.size() << endl;
    pcl::PCDWriter writer;
    // writer.write("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_UniSmp.pcd", *cloud_filtered, true);

    //---------------------显示点云-----------------------
    visualize_cloud(cloud, cloud_filtered);

    return 0;
}

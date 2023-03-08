#include <iostream>
#include <Car_Spraying/MeanFiter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
using namespace std;

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_Re_VG.pcd", *cloud);
    MeanFilter mf;
    mf.setInputCloud(cloud);
    // mf.setKSearch(20);
    mf.setRadiusSearch(0.05);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>);
    mf.filter(*cloudOut);
    pcl::io::savePCDFileASCII("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_Re_VG_MF.pcd", *cloudOut);

    // ------------------------------可视化---------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("ShowCloud"));
    view->setWindowName("点云中值滤波");
    int v1(0);
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    view->setBackgroundColor(0, 0, 0, v1);
    view->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    view->createViewPort(0.5, 0.0, 1, 1.0, v2);
    view->setBackgroundColor(0.1, 0.1, 0.1, v2);
    view->addText("filtered point clouds", 10, 10, "v2_text", v2);

    view->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud", v1);
    view->addPointCloud<pcl::PointXYZRGB>(cloudOut, "cloud_filtered", v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    // view->addCoordinateSystem(1.0);
    // view->initCameraParameters();
    while (!view->wasStopped())
    {
        view->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h> // 体素中心点滤波
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main(int argc, char **argv)
{
    // --------------------------------加载点云----------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De.pcd", *cloud);
    cout << "从点云中读取 " << cloud->size() << " 个点" << endl;
    // ---------------------------ApproximateVoxelGrid---------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> avf;
    avf.setInputCloud(cloud);          // 输入点云
    avf.setLeafSize(0.05, 0.05, 0.05); // 最小体素的边长
    avf.filter(*filtered_cloud);       // 进行滤波
    pcl::io::savePCDFileASCII ("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_AVG.pcd", *filtered_cloud); // 保存滤波结果
    cout << "体素中心点滤波完毕！！！" << endl;
    cout << "滤波后点的个数为：" << filtered_cloud->size() << endl;
    // ---------------------------------结果可视化-------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowCloud"));

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("Raw point clouds", 10, 10, "v1_text", v1);
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1, 1.0, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->addText("filtered point clouds", 10, 10, "v2_text", v2);
    viewer->setWindowName("ApproximateVoxelGrid");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud", v1);
    viewer->addPointCloud<pcl::PointXYZRGB>(filtered_cloud, "cloud_filtered", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_filtered", v2);
    // view->addCoordinateSystem(1.0);
    // view->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}

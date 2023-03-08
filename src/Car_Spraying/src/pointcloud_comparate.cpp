#include <string>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
using namespace std;
using namespace pcl;
typedef pcl::PointXYZ PointType;
//创建了一个名为cloud的指针，储存XYZ类型的点云数据
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
PointType pointSel;
class PcdRead
{
public:
    void readPcdFile(const std::string filePath);
};
void PcdRead::readPcdFile(const string filePath)
{

    /*打开点云文件*/
    if (io::loadPCDFile<PointXYZ>(filePath, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file xxx.pcd\n");
    }
}
void point1To2(PointType const *const p1, PointType *const p2)
{
    p2->x = p1->x * 0.0 + p1->y * 0.0 + p1->z * 0.0;
    p2->y = p1->x * 0.0 + p1->y * 0.0 + p1->z * 0.0;
    p2->z = p1->x * 0.0 + p1->y * 0.0 + p1->z * -0.1;
}
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> index;
    // Fill in the CloudIn data
    PcdRead read;
    read.readPcdFile("./output_1.pcd");
    // pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
    *cloud_in = *cloud;
    std::cout << "Saved " << cloud_in->size() << " data points from output_1.pcd:" << std::endl;
    read.readPcdFile("./output_2.pcd");
    // pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
    *cloud_out = *cloud;
    std::cout << "Saved " << cloud_in->size() << " data points from output_2.pcd:" << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        point1To2(&cloud_in->points[i], &pointSel); //雷达1数据转换到雷达2坐标系中
        cloud_out->push_back(pointSel);
    }
    std::cout << "雷达1数据转换到雷达2坐标系中" << std::endl;
    /*点云pcd文件可视化*/
    visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud(cloud_out, "rabbit-cloud");
    viewer->spin();
    return (0);
}

#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB.pcd", *cloud); 
    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*cloud, mesh.cloud);
    pcl::io::saveOBJFile("/home/lzq/lzq_ws/pointcloud/3.1/XYZ.obj", mesh);

    return 0;
}
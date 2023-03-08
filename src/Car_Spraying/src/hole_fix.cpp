// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/PCLPointCloud2.h>
// #include <iostream>
// #include <string>

// using namespace pcl;
// using namespace pcl::io;
// using namespace std;

// int PCDtoPLYconvertor(string &input_filename, string &output_filename)
// {
//     pcl::PCLPointCloud2 cloud;
//     if (loadPCDFile(input_filename, cloud) < 0)
//     {
//         cout << "Error: cannot load the PCD file!!!" << endl;
//         return -1;
//     }
//     PLYWriter writer;
//     writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
//     return 0;
// }

// int main()
// {
//     string input_filename = "/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB.pcd";
//     string output_filename = "/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB.ply";
//     PCDtoPLYconvertor(input_filename, output_filename);
//     return 0;
// }
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter_indices.h>

int main(int argc, char const *argv[])
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/XYZRGB_Re_VG_MF_SOR.pcd", *cloud);

    // Remove any NaN values from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(0.15);   // Radius of the sphere for each point
    mls.setPolynomialFit(true); // Fit a polynomial to the points
    mls.setPolynomialOrder(3);  // Order of the polynomial fit
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::UpsamplingMethod::SAMPLE_LOCAL_PLANE); // Upsampling method
    mls.setUpsamplingRadius(0.008);                                                                                              // Radius of the sphere for each upsampled point
    mls.setUpsamplingStepSize(0.005);                                                                                            // Step size for each upsampled point

    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::UpsamplingMethod::VOXEL_GRID_DILATION); // Upsampling method
    // mls.setDilationVoxelSize(0.05);
    // mls.setDilationIterations(10);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    mls.process(*mls_cloud);
    pcl::io::savePCDFile<pcl::PointXYZRGB>("/home/lzq/lzq_ws/pointcloud/3.1/hole_fix.pcd", *mls_cloud);

    // pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    // viewer.setBackgroundColor(0.0, 0.0, 0.0);
    // viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "original_cloud");
    // viewer.addPointCloud<pcl::PointXYZRGB>(mls_cloud, "mls_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "mls_cloud");
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("滤波前后对比"));

    /*-----视口1-----*/
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
    viewer->setBackgroundColor(0, 0, 0, v1);        // 设置背景颜色，0-1，默认黑色（0，0，0）
    viewer->addText("befor_filtered", 10, 10, "v1_text", v1);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "befor_filtered_cloud", v1);

    /*-----视口2-----*/
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer->addText("after_filtered", 10, 10, "v2_text", v2);
    viewer->addPointCloud<pcl::PointXYZRGB>(mls_cloud, "after_filtered_cloud", v2);

    /*-----设置相关属性-----*/
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "befor_filtered_cloud", v1);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "befor_filtered_cloud", v1);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "after_filtered_cloud", v2);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "after_filtered_cloud", v2);

    viewer->spin();

    return 0;
}
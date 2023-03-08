#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>

int main()
{
    // Read PCD file
    std::string input_file_name = "/home/lzq/lzq_ws/pointcloud/11.30/sub_0.01.pcd";
    // std::cout << "Enter the input file name: ";
    // std::cin >> input_file_name;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file_name, *cloud) == -1)
    {
        PCL_ERROR("Could not read PCD file\n");
        return (-1);
    }

    // Write OFF file
    std::string output_file_name = "/home/lzq/lzq_ws/pointcloud/11.30/out.off";
    // std::cout << "Enter the output file name: ";
    // std::cin >> output_file_name;

    std::ofstream off_file(output_file_name);
    off_file << "COFF\n";
    off_file << cloud->size() << " 0 0\n";
    for (const auto &point : *cloud)
    {
        off_file << point.x << " " << point.y << " " << point.z << " "
                 << static_cast<int>(point.r) << " " << static_cast<int>(point.g) << " "
                 << static_cast<int>(point.b) << " 255\n";
    }
    off_file.close();

    return 0;
}
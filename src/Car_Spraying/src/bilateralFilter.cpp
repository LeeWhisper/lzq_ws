#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

inline double kernel(double x, double sigma)
{
    return (std::exp(-(x * x) / (2 * sigma * sigma)));
}

// 计算法向量
pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target_cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> n;
    n.setInputCloud(target_cloud);
    n.setSearchMethod(tree);
    n.setKSearch(10);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.compute(*normals);

    return normals;
}

int main(int argc, char *argv[])
{
    std::string incloudfile = "/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_UniSmp.pcd";
    std::string outcloudfile = "sphere_noisy_bffilter.pcd";

    // ---------------------------------加载点云---------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(incloudfile.c_str(), *cloud);
    std::cout << "\t\t<读入点云信息>\n"
              << *cloud << std::endl;
    // -------------------------------设置参数阈值-------------------------------------
    float sigma_s = 0.05;
    float sigma_r = 5;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr BFcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    BFcloud = cloud;
    // -------------------------------建立KD树索引----------------------------------
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud);

    pcl::Indices k_indices;
    std::vector<float> k_distances;
    // ---------------------------基于法线的双边滤波--------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr normals_input = computeNormal(cloud);

    for (int point_id = 0; point_id < cloud->size(); ++point_id)
    {
        float BF = 0;
        float W = 0;

        tree->radiusSearch(point_id, 2 * sigma_s, k_indices, k_distances);
        Eigen::Vector3f normal = (*normals_input)[point_id].getNormalVector3fMap();
        // 遍历每一个点
        for (std::size_t n_id = 0; n_id < k_indices.size(); ++n_id)
        {
            int id = k_indices.at(n_id);
            float dist = sqrt(k_distances.at(n_id)); // 计算欧氏距离

            Eigen::Vector3f point_p = cloud->points[point_id].getVector3fMap(),
                            point_q = cloud->points[k_indices[n_id]].getVector3fMap();
            float normal_dist = normal.dot(point_q - point_p); // 计算法线距离
            // 计算高斯核函数
            float w_a = kernel(dist, sigma_s);
            float w_b = kernel(normal_dist, sigma_r);
            float weight = w_a * w_b; // w

            BF += weight * normal_dist; // sum_l
            W += weight;                // sum_w
        }
        // 滤波之后的点
        Eigen::Vector3f point_filter = cloud->points[point_id].getVector3fMap() + (BF / W) * normal;
        BFcloud->points[point_id].x = point_filter[0];
        BFcloud->points[point_id].y = point_filter[1];
        BFcloud->points[point_id].z = point_filter[2];
    }
    std::cout << "\t\t<保存点云信息>\n"
              << *BFcloud << std::endl;
    // pcl::io::savePCDFile(outcloudfile.c_str(), *BFcloud);

    //================================= 滤波前后对比可视化 ================================= ↓

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
    viewer->addPointCloud<pcl::PointXYZRGB>(BFcloud, "after_filtered_cloud", v2);

    /*-----设置相关属性-----*/
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "befor_filtered_cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "befor_filtered_cloud", v1);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "after_filtered_cloud", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "after_filtered_cloud", v2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}

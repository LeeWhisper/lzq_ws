#include <Car_Spraying/MeanFiter.h>
#include <pcl/kdtree/kdtree_flann.h> //kdtree近邻搜索
#include <pcl/common/eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>

bool MeanFilter::setInputCloud(const pcl::PointCloud<PointT>::Ptr &input_cloud)
{
    m_input = input_cloud;

    return true;
}

void MeanFilter::filter(pcl::PointCloud<PointT> &m_output)
{
    if (search_radius_ != 0.0)
    {
        if (k_ != 0)
        {
            printf("Both radius and K defined!,Set one of them to zero first and then re-run compute ().\n");
            exit(10);
        }
    }

    if (k_ != 0)
    {
        pcl::KdTreeFLANN<PointT> kdtree; // 建立kdtree对象
        kdtree.setInputCloud(m_input);
        std::vector<int> neighbor_index_k;             // 领域索引
        std::vector<float> neighbor_square_distance_k; // 领域距离大小

        PointT search_point;

        for (size_t i = 0; i < m_input->points.size(); ++i)
        {
            search_point = m_input->points[i];

            if (kdtree.nearestKSearch(search_point, k_, neighbor_index_k, neighbor_square_distance_k) > 0) // 寻找领域为K的值
            {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*m_input, neighbor_index_k, centroid); // 计算质心
                // pcl::PointXYZRGB points = {centroid[0], centroid[1], centroid[2]};
                pcl::PointXYZRGB points;
                points.x = centroid[0];
                points.y = centroid[1];
                points.z = centroid[2];
                points.r = m_input->points[i].r;
                points.g = m_input->points[i].g;
                points.b = m_input->points[i].b;

                m_output.push_back(points);
            }
        }
    }

    else if (search_radius_ != 0.0)
    {
        pcl::KdTreeFLANN<PointT> kdtree; // 建立kdtree对象
        kdtree.setInputCloud(m_input);
        std::vector<int> neighbor_index_r;             // 领域索引
        std::vector<float> neighbor_square_distance_r; // 领域距离大小

        PointT search_point;

        for (size_t i = 0; i < m_input->points.size(); ++i)
        {
            search_point = m_input->points[i];

            if (kdtree.radiusSearch(search_point, search_radius_, neighbor_index_r, neighbor_square_distance_r) > 0) // 寻找领域为K的值
            {
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*m_input, neighbor_index_r, centroid); // 计算质心
                // pcl::PointXYZRGB points = {centroid[0], centroid[1], centroid[2]};
                pcl::PointXYZRGB points;
                points.x = centroid[0];
                points.y = centroid[1];
                points.z = centroid[2];
                points.r = m_input->points[i].r;
                points.g = m_input->points[i].g;
                points.b = m_input->points[i].b;

                m_output.push_back(points);
            }
            std::cout << i << "/" << m_input->points.size() << std::endl;
        }
    }
    else
    {
        std::cout << "检查参数设置是否正确" << std::endl;
    }
}

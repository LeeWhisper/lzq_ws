#pragma once
#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/point_types.h>  //点类型相关定义

using PointT = pcl::PointXYZRGB;
class MeanFilter
{
public:
	bool setInputCloud(const pcl::PointCloud<PointT>::Ptr& input_cloud);
	
	inline void
		setKSearch(int k) { k_ = k; }

	inline void
		setRadiusSearch(double radius)
	{
		search_radius_ = radius;
	}

	void filter(pcl::PointCloud<PointT>& m_output);

private:
	pcl::PointCloud<PointT>::Ptr m_input;

	int k_ = 0;
	float search_radius_ = 0.0;
};


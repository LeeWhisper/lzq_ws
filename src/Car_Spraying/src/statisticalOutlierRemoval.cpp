#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ctime>

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);		  // 待滤波点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); // 滤波后点云
	long startTime, endTime;

	/// 读入点云数据
	cout << "->正在读入点云..." << endl;
	pcl::PCDReader reader;
	reader.read("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_UniSmp.pcd", *cloud);
	cout << "\t\t<读入点云信息>\n"
		 << *cloud << endl;

	/// 统计滤波
	cout << "->正在进行统计滤波..." << endl;
	startTime = clock();

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; // 创建滤波器对象
	sor.setInputCloud(cloud);							  // 设置待滤波点云
	sor.setMeanK(6);									  // 设置查询点近邻点的个数
	sor.setStddevMulThresh(1.5);						  // 设置标准差乘数，来计算是否为离群点的阈值
	// sor.setNegative(true);							//默认false，保存内点；true，保存滤掉的离群点
	sor.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered

	float min_height = 0.0, min_width = 0.0, max_width = 0.0, width;
	for (int i = 0; i < cloud_filtered->points.size(); i++)
	{
		if (max_width < cloud_filtered->points[i].y)
		{
			max_width = cloud_filtered->points[i].y;
		}
		if (min_width > cloud_filtered->points[i].y)
		{
			min_width = cloud_filtered->points[i].y;
		}
	}
	width = max_width - min_width;
	std::cout << "width: " << width << std::endl;

	/// 保存下采样点云
	cout << "->正在保存滤波点云..." << endl;
	pcl::PCDWriter writer;
	// writer.write("/home/lzq/lzq_ws/pointcloud/3.1/Hf_De_UniSmp_SOR.pcd", *cloud_filtered, true);
	cout << "\t\t<保存点云信息>\n"
		 << *cloud_filtered << endl;
	endTime = clock();
	std::cout << "run time: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
	//================================= 滤波前后对比可视化 ================================= ↓

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("滤波前后对比"));

	/*-----视口1-----*/
	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // 设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
	viewer->setBackgroundColor(0, 0, 0, v1);		// 设置背景颜色，0-1，默认黑色（0，0，0）
	viewer->addText("befor_filtered", 10, 10, "v1_text", v1);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "befor_filtered_cloud", v1);

	/*-----视口2-----*/
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("after_filtered", 10, 10, "v2_text", v2);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "after_filtered_cloud", v2);

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

	//================================= 滤波前后对比可视化 ================================= ↑

	return 0;
}

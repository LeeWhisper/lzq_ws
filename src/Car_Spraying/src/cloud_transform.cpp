//****变换点云位置****//

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("./output_1.pcd", *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);

	//Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity();//定义绕X轴的旋转矩阵，并初始化为单位阵
	//double angle_x = M_PI / 2;//旋转90°
	//rotation_x(1, 1) = cos(angle_x);
	//rotation_x(1, 2) = -sin(angle_x);
	//rotation_x(2, 1) = sin(angle_x);
	//rotation_x(2, 2) = cos(angle_x);
	//pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_x);

	//Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();//定义绕Y轴的旋转矩阵，并初始化为单位阵
	//double angle_y = M_PI / 2;//旋转90°
	//rotation_y(0, 0) = cos(angle_y);
	//rotation_y(0, 2) = sin(angle_y);
	//rotation_y(2, 0) = -sin(angle_y);
	//rotation_y(2, 2) = cos(angle_y);
	//pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_y);

	//Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();//定义绕Z轴的旋转矩阵，并初始化为单位阵
	//double angle_z = M_PI / 2;//旋转90°
	//rotation_z(0, 0) = cos(angle_z);
	//rotation_z(0, 1) = -sin(angle_z);
	//rotation_z(1, 0) = sin(angle_z);
	//rotation_z(1, 1) = cos(angle_z);
	//pcl::transformPointCloud(*cloud, *cloud_transformed, rotation_z);

	Eigen::Vector4f cloudCentroid;
	pcl::compute3DCentroid(*cloud, cloudCentroid);//计算点云质心
	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();//定义平移矩阵，并初始化为单位阵
	// translation(0, 3) = -cloudCentroid[0];
	// translation(1, 3) = -cloudCentroid[1];
	// translation(2, 3) = -cloudCentroid[2];
	translation(2, 3) = -1;
	pcl::transformPointCloud(*cloud, *cloud_transformed, translation);
	
	//for (int i = 0; i < cloud->size(); i++)//点云中的每个点减去质心，也可实现平移
	//{
	//	pcl::PointXYZ temp;
	//	temp.x = cloud->points[i].x - cloud_Centroid[0];
	//	temp.y = cloud->points[i].y - cloud_Centroid[1];
	//	temp.z = cloud->points[i].z - cloud_Centroid[2];
	//	cloud_transformed->push_back(temp);
	//}

	//可视化
	pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
	viewer.initCameraParameters();

	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.addCoordinateSystem(0.5, "reference", v1);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
	viewer.addText("Cloud before transforming", 10, 10, "v1 test", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0, 255, 0);
	viewer.addPointCloud(cloud, color, "cloud", v1);

	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer.addCoordinateSystem(0.5,"reference", v2);
	viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
	viewer.addText("Cloud after transforming", 10, 10, "v2 test", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_transformed(cloud_transformed, 0, 255, 0);
	viewer.addPointCloud(cloud_transformed, color_transformed, "cloud_transformed", v2);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}

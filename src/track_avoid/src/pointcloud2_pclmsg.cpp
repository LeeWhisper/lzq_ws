#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
// #include <pcl/registration/icp.h>   //ICP配准类相关头文件
#include <pcl/visualization/cloud_viewer.h>
#include <eigen3/Eigen/Dense>
#include <pcl/ModelCoefficients.h> // 模型系数定义的头文件
#include <pcl/filters/project_inliers.h>   // 投影滤波的头文件
#include <pcl/filters/passthrough.h>
using namespace Eigen;
ros::Publisher pub;
void callback(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr)
{
    // cout<<"00000000000000"<<endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);

    // pcl_conversions::toPCL(*in_cloud_ptr,pcl_pc2);
    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*in_cloud_ptr,pcl_pc2);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr now_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(pcl_pc2,*pclcloud);
    pcl::fromROSMsg (*in_cloud_ptr, *pclcloud);
    // cout<<"pclcloud:"<<pclcloud->size()<<endl;
    // 高度滤波
    // 创建滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;//创建滤波器对象
    pass.setInputCloud (pclcloud);			//设置待滤波的点云
    // pass.setFilterFieldName ("z");		//设置在Z轴方向上进行滤波
    pass.setFilterFieldName ("y");		//设置在Z轴方向上进行滤波

    // 坐标轴方向不同z, -x, -y对应着cloud_projected中的x，y，z
    pass.setFilterLimits (-2,0.8);		//设置滤波范围为0~1,在范围之外的点会被剪除
    // pass.setFilterLimits (0,1);		//设置滤波范围为0~1,在范围之外的点会被剪除

    //pass.setFilterLimitsNegative (true);//是否反向过滤，默认为false
    pass.filter (*cloud_projected);		//开始过滤
    // cout<<"cloud_projected:"<<cloud_projected->size()<<endl;
    // for (size_t i = 0; i < cloud_projected->points.size (); ++i)
    // // std::cerr << "    " << cloud_projected->points[i].x << " " 
    // //                     << cloud_projected->points[i].y << " " 
    // //                     << cloud_projected->points[i].z << std::endl;

	// for(int i=0;i<in_cloud_ptr->width*in_cloud_ptr->height;i++)
    // {
    //     pcl::PointXYZ p;
    //     std::memcpy(&p.x,&in_cloud_ptr->data[32*i],4);
    //     std::memcpy(&p.y,&in_cloud_ptr->data[32*i+4],4);
    //     std::memcpy(&p.z,&in_cloud_ptr->data[32*i+8],4);
    //     std::memcpy(&p.rgba,&in_cloud_ptr->data[32*i+16],4);
    //     pclcloud->points.push_back(p);
    // }
    // 填充　ModelCoefficients 的值
    // ax + by + cz + d = 0, 其中 a = 0, b = 0, d = 0,  c = 1, 换句话说就是　X-Y 平面
    // cout<<"111111111111"<<endl;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 1.0;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;
    // cout<<"222222222222"<<endl;

    // 创建 projectInliers 对象， 并使用刚才定义好的 ModelCoefficients 作为投影对象模型参数
    pcl::ProjectInliers<pcl::PointXYZ> proj;            //创建投影滤波
    proj.setModelType(pcl::SACMODEL_NORMAL_PLANE);      // 设置对象对应的投影模型
    proj.setInputCloud(cloud_projected);                          // 设置输入点云
    proj.setModelCoefficients(coefficients);            // 设置模型对应的系数
    proj.filter(*result);                      // 执行投影滤波存储结果到 cloud_projected
    // 
    // pcl::PointCloud<pcl::PointXYZ> cloud_pcl_xyz;
    // cout<<"33333333333333"<<endl;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*result,cloud_msg);
    pub.publish(cloud_msg);

}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_conversion");
    ros::NodeHandle ph;
    pub = ph.advertise<sensor_msgs::PointCloud2>("projected_pcl",1, true);
    ros::Subscriber sub = ph.subscribe("/camera/depth/color/points", 1, &callback);
    ros::spin(); 
    return 0;
}
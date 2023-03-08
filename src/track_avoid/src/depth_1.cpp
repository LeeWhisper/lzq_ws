#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;

class ImageConverter
{
    private:
    // 创建句柄
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    // 接受RGB图像话题
    image_transport::Subscriber image_sub_color;
    // 接受深度图像话题
    image_transport::Subscriber image_sub_depth;
    // 接受深度图像对应的相机参数话题
    ros::Subscriber camera_info_sub_;
    // 发布一个三维座标点，可用于可视化
    ros::Publisher maker_point_pub_;

    sensor_msgs::CameraInfo camera_info;
    geometry_msgs::PointStamped output_point;

    Mat colorImage;
    // 注意这里修改为你接受的深度图像尺寸
    Mat depthImage = Mat::zeros(480, 640, CV_16UC1);
    // 鼠标点的初始点位(0,0)
    Point mousepos = Point(0, 0);

    public:
    // 获取鼠标的坐标，通过param指针传出到类成员Point mousepos
    static void on_mouse(int event, int x, int y, int flags, void *param)
    {
        switch (event)
        {
        case cv::EVENT_LBUTTONDOWN:
        {
            Point &tmppoint = *(cv::Point *) param;
            tmppoint = Point(x, y);
        }
        break;
        }
    }

    ImageConverter():it_(nh_)
    {
        // 接受深度图像
        image_sub_depth = it_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ImageConverter::imageDepthCb, this);
        // 接受RGB图像
        image_sub_color = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageColorCb, this);
        // 接受深度图像至RBG图像的坐标转换话题
        camera_info_sub_ = nh_.subscribe( "/camera/aligned_depth_to_color/camera_info", 1, &ImageConverter::cameraInfoCb, this);
        // 发送mouse话题
        maker_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/mouse_point", 10);
        cv::namedWindow("colorImage");
        // 窗口、回调函数、参数
        setMouseCallback("colorImage", &ImageConverter::on_mouse, (void*) &mousepos);
    }

    ~ImageConverter()
    {
        cv::destroyWindow("colorImage");
    }

    void cameraInfoCb(const sensor_msgs::CameraInfo &msg)
    {
        camera_info = msg;
    }

    void imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try 
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1 );
			depthImage = cv_ptr->image;
		} 
        catch ( cv_bridge::Exception &e ) 
        {
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
        }
    }

    void imageColorCb( const sensor_msgs::ImageConstPtr &msg )
	{
		cv_bridge::CvImagePtr cv_ptr;
		try 
        {
			cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
			colorImage = cv_ptr->image;
		} 
        catch(cv_bridge::Exception &e)
        {
			ROS_ERROR( "cv_bridge exception: %s", e.what());
			return;
		}
		
		//先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
		float real_z = 0.001 * depthImage.at<u_int16_t>( mousepos.y, mousepos.x );
		float real_x = (mousepos.x - camera_info.K.at( 2 ) ) / camera_info.K.at( 0 ) * real_z;
		float real_y = (mousepos.y - camera_info.K.at( 5 ) ) / camera_info.K.at( 4 ) * real_z;

		char tam[100];
		sprintf(tam,"(%0.2f,%0.2f,%0.2f)", real_x, real_y, real_z);
        cout << "x:"<<real_x<<",y:"<<real_y<<",z:"<<real_z<<","<<endl;
        //打印到屏幕上
		putText(colorImage, tam, mousepos, FONT_HERSHEY_SIMPLEX, 0.6, cvScalar( 0, 0, 255 ), 1);
		circle( colorImage, mousepos, 2, Scalar( 255, 0, 0 ) );
		output_point.header.frame_id = "/camera_depth_optical_frame";
		output_point.header.stamp = ros::Time::now();
		output_point.point.x = real_x;
		output_point.point.y = real_y;
		output_point.point.z = real_z;
		maker_point_pub_.publish(output_point);
		cv::imshow( "colorImage", colorImage);
		cv::waitKey(1);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "coordinate_map");
    ImageConverter imageconverter;
    ros::spin();
    return(0);
}
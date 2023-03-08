#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/PointCloud2.h>
#include "image_transport/image_transport.h"

using namespace std;

class depth_avoid
{
private:
    ros::NodeHandle nh;
    // image_transport::Subscriber sub_depth;
    ros::Subscriber sub_depth;
    
    void depth_callback(const sensor_msgs::PointCloud2 &depth);

public:
    cv_bridge::CvImagePtr depth_ptr;
    cv::Mat depth_img;
    
    depth_avoid();
};

depth_avoid::depth_avoid()
{
    // image_transport::ImageTransport it(nh);
    sub_depth = nh.subscribe("/projected_pcl", 1, &depth_avoid::depth_callback, this);
}

//深度回调函数 
void depth_avoid::depth_callback(const sensor_msgs::PointCloud2 &depth)
{
    // depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16SC1);
    // depth_img = depth_ptr->image;
    // float min_depth;
    // min_depth = depth_img.at<uint16_t>(0, 0);
    // for (size_t v = 0; v < depth_img.rows; v++)
    // {
    //     for (size_t u = 0; u < depth_img.cols; u++)
    //     {
    //         if (depth_img.at<uint16_t>(v, u) < min_depth)
    //         min_depth = depth_img.at<uint16_t>(v, u);
    //     }
    // }
    // cout << "min_depth: " << min_depth << endl;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "depth_avoid");
    depth_avoid de_avoid;

    ros::spin();
    return 0;
}

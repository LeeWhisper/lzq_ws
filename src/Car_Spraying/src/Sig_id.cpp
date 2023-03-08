#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>
#include <iostream>

class Sig
{
private:
    ros::NodeHandle nh;
    image_transport::Subscriber sub_img;
    cv::String saliency_algorithm = "SPECTRAL_RESIDUAL";

public:
    cv::Mat image, binaryMap;
    cv_bridge::CvImagePtr color_ptr;
    cv::Ptr<cv::saliency::Saliency> saliencyAlgorithm;

    Sig();
    void image_callback(const sensor_msgs::ImageConstPtr &img);
};

Sig::Sig()
{
    image_transport::ImageTransport it(nh);
    sub_img = it.subscribe("/camera/color/image_raw", 1, &Sig::image_callback, this);
}

void Sig::image_callback(const sensor_msgs::ImageConstPtr &img)
{
    cv::Mat frame;
    color_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    frame = color_ptr->image;
    // frame.copyTo(image);
    if (frame.empty())
        if (frame.channels() == 4)
            cvtColor(frame, frame, cv::COLOR_BGRA2BGR);

    cv::imshow("Image", frame);
    cv::waitKey(10);
    // if (saliency_algorithm.find("SPECTRAL_RESIDUAL") == 0)
    // {
    //     // 检测结果，白色区域表示显著区域
    //     cv::Mat saliencyMap;
    //     saliencyAlgorithm = cv::saliency::StaticSaliencySpectralResidual::create();
    //     // 计算显著性
    //     bool success = saliencyAlgorithm->computeSaliency(frame, saliencyMap);
    //     std::cout << success << std::endl;
    //     if (success)
    //     {
    //         cv::saliency::StaticSaliencySpectralResidual spec;
    //         // 二值化图像
    //         spec.computeBinaryMap(saliencyMap, binaryMap);

    //         cv::imshow("Original Image", image);
    //         cv::imshow("Saliency Map", saliencyMap);
    //         cv::imshow("Binary Map", binaryMap);

    //         cv::waitKey(10);
    //     }
    // }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Sig_id");
    Sig sig;

    ros::spin();
    return 0;
}

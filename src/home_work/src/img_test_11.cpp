#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "img_test_11");
    cv::Mat img, hsv, mask, mask_1, mask_2, img_mask;
    img = cv::imread("/home/lzq/车门项目/Picture/output_1.png");
    //转为HSV空间
    cv::cvtColor(img, hsv, CV_BGR2HSV);
    //一次掩膜
    cv::inRange(hsv, cv::Scalar(15, 150, 150), cv::Scalar(40, 255, 255), mask);
    //二次掩膜
    // cv::inRange(hsv, cv::Scalar(0, 0, 125), cv::Scalar(30, 255, 255), mask_2);
    //或操作，掩膜相加
    // mask = mask_1 | mask_2;
    //获取自定义核
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    //进行膨胀操作
    dilate(mask, mask, element);
    //增大自定义核，进行腐蚀操作
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    cv::erode(mask, mask, element);

    img.copyTo(img_mask, mask);
    while (ros::ok())
    {
        cv::imshow("frame", img_mask);
        // cv::imwrite("/home/lzq/lzq_ws/src/home_work/piture/rose_out.png", img_mask);
        cv::waitKey(10);
    }
    ros::spin();
    return 0;
}

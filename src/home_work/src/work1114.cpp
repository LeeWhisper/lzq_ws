#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <opencv2/saliency.hpp>
#include <opencv2/imgproc/types_c.h>
#include <iostream>

void detectHSColor(const cv::Mat &image, double minHue,
                   double maxHue, double minSat,
                   double maxSat, double minVal,
                   double maxVal, cv::Mat &mask)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(minHue, minSat, minVal), cv::Scalar(maxHue, maxSat, maxVal), mask);
    // std::vector<cv::Mat> channels;
    // cv::split(hsv, channels);
    // cv::Mat mask1, mask2, hueMask;
    // cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
    // cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
    // if (minHue < maxHue)
    // {
    //     hueMask = mask1 & mask2;
    // }
    // else
    // {
    //     hueMask = mask1 | mask2;
    // }
    // cv::Mat satMask;
    // cv::inRange(channels[1], minSat, maxSat, satMask);
    // mask = hueMask & satMask;
    // cv::imshow("mask", mask);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "work1115");
    cv::Mat img, hsv, mask, mask_1, mask_2, img_mask;
    img = cv::imread("/home/lzq/lzq_ws/src/home_work/piture/rose.jpg");
    //转为HSV空间
    cv::cvtColor(img, hsv, CV_BGR2HSV);
    //一次掩膜
    cv::inRange(hsv, cv::Scalar(80, 0, 125), cv::Scalar(180, 255, 255), mask_1);
    //二次掩膜
    cv::inRange(hsv, cv::Scalar(0, 0, 125), cv::Scalar(30, 255, 255), mask_2);
    //或操作，掩膜相加
    mask = mask_1 | mask_2;
    //获取自定义核
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
    //进行膨胀操作
    dilate(mask, mask, element);
    //增大自定义核，进行腐蚀操作
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(40, 40));
    cv::erode(mask, mask, element);

    img.copyTo(img_mask, mask);
    while (ros::ok())
    {
        cv::imshow("frame", img_mask);
        cv::imwrite("/home/lzq/lzq_ws/src/home_work/piture/rose_out.png", img_mask);
        cv::waitKey(0);
    }
    ros::spin();
    return 0;
}

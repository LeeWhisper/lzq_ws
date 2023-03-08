#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <stack>

using namespace std;
using namespace cv;

//比较轮廓面积(USB_Port_Lean用来进行轮廓排序)
bool Contour_Area(vector<Point> contour1, vector<Point> contour2)
{
    return contourArea(contour1) > contourArea(contour2);
}

void detectHSColor(const cv::Mat &image, double minHue,
                   double maxHue, double minSat,
                   double maxSat, cv::Mat &mask)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    // cv::inRange(hsv, cv::Scalar(minHue, minSat, minVal), cv::Scalar(maxHue, maxSat, maxVal), mask);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat mask1, mask2, hueMask;
    cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
    cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
    if (minHue < maxHue)
    {
        hueMask = mask1 & mask2;
    }
    else
    {
        hueMask = mask1 | mask2;
    }
    cv::Mat satMask;
    cv::inRange(channels[1], minSat, maxSat, satMask);
    mask = hueMask & satMask;
    // cv::imshow("mask", mask);
}

//回调函数
void imageCalllback(const cv::Mat img)
{
    cv::Mat frame, mask, out, image;
    // cv_bridge::CvImagePtr color_ptr;
    // color_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    // frame = color_ptr->image;

    // frame.copyTo(image);
    // cv::imshow("frame", frame);
    // detectHSColor(frame, 10, 40, 130, 255, mask);
    // frame.copyTo(out, mask);
    // cv::imshow("output", out);

    Mat src;
    cvtColor(img, src, COLOR_BGR2GRAY);

    Mat thresh;
    threshold(src, thresh, 0, 120, THRESH_OTSU);

    std::vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

    cv::findContours(thresh, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE, Point());
    std::cout << "Contours: " << contours.size() << std::endl;

    //面积最大的在前面
    sort(contours.begin(), contours.end(), Contour_Area);
    //存储前三个最大的多边形
    vector<vector<Point>> contours_poly(3);
    for (int i = 0; i < 3; i++)
    {
        double area = contourArea(contours[i]);
        cout << area << endl;
        //逼近多边形曲线
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
    }
    // 画多边形轮廓
    //中间轮廓
    cv::Mat imageContours = cv::Mat::zeros(img.size(), CV_8UC1);
    drawContours(imageContours, contours_poly, 2, Scalar(255, 255, 255), 1, 8);

    //最外轮廓
    cv::Mat Contours = cv::Mat::zeros(img.size(), CV_8UC1); //绘制边界矩形
    drawContours(Contours, contours_poly, 0, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
    while (ros::ok())
    {
        /* code */
        cv::imshow("最大轮廓", Contours);
        cv::imshow("中间轮廓", imageContours);
    }

    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    // image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 10, imageCalllback);
    cv::Mat img;
    img = cv::imread("/home/lzq/车门项目/Picture/output_1.png");
    cv::Mat hsv, mask, mask_1, mask_2, img_mask;

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

    cv::Mat frame, out, image;
    Mat src;
    cvtColor(img_mask, src, COLOR_BGR2GRAY);

    Mat thresh;
    threshold(src, thresh, 0, 255, THRESH_OTSU);

    std::vector<vector<Point>> contours;
    std::vector<Vec4i> hierarchy;

    cv::findContours(src, contours, hierarchy, RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0));
    std::cout << "Contours: " << contours.size() << std::endl;

    //面积最大的在前面
    sort(contours.begin(), contours.end(), Contour_Area);
    //存储前三个最大的多边形
    vector<vector<Point>> contours_poly(3);
    for (int i = 0; i < 3; i++)
    {
        double area = contourArea(contours[i]);
        cout << area << endl;
        //逼近多边形曲线
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
    }
    // 画多边形轮廓
    //中间轮廓
    cv::Mat imageContours = cv::Mat::zeros(img.size(), CV_8UC1);
    drawContours(imageContours, contours_poly, 2, Scalar(255, 255, 255), 1, 8);

    //最外轮廓
    cv::Mat Contours = cv::Mat::zeros(img.size(), CV_8UC1); //绘制边界矩形
    drawContours(Contours, contours_poly, 0, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
    while (ros::ok())
    {
        /* code */
        cv::imshow("frame", img_mask);
        cv::imshow("最大轮廓", Contours);
        cv::imshow("中间轮廓", imageContours);

        cv::waitKey(10);
    }

    ros::spin();

    return 0;
}

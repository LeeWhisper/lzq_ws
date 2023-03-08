//---------------------------------【头文件、命名空间包含部分】----------------------------
//        描述：包含程序所使用的头文件和命名空间
//------------------------------------------------------------------------------------------------
// #include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>

#include <iostream>
using namespace cv;
using namespace std;

//-----------------------------------【宏定义部分】--------------------------------------------
//        描述：定义一些辅助宏
//------------------------------------------------------------------------------------------------
#define WINDOW_NAME1 "【原始图窗口】" //为窗口标题定义的宏
#define WINDOW_NAME2 "【轮廓图】"     //为窗口标题定义的宏

//-----------------------------------【全局变量声明部分】--------------------------------------
//        描述：全局变量的声明
//-----------------------------------------------------------------------------------------------
Mat g_srcImage, hsv, mask, img_mask;
Mat g_grayImage;
int g_nThresh = 80;
int g_nThresh_max = 255;
RNG g_rng(12345);
Mat g_cannyMat_output;
vector<vector<Point>> g_vContours;
vector<Vec4i> g_vHierarchy;

//-----------------------------------【全局函数声明部分】--------------------------------------
//        描述：全局函数的声明
//-----------------------------------------------------------------------------------------------
static void ShowHelpText();
void on_ThreshChange(int, void *);

//-----------------------------------【main( )函数】--------------------------------------------
//        描述：控制台应用程序的入口函数，我们的程序从这里开始执行
//-----------------------------------------------------------------------------------------------
bool Contour_Area(vector<Point> contour1, vector<Point> contour2)
{
    return contourArea(contour1) > contourArea(contour2);
}

int main(int argc, char **argv)
{
    //【0】改变console字体颜色
    system("color 1F");

    // 加载源图像
    g_srcImage = imread("/home/lzq/车门项目/Picture/output_2.png", 1);
    if (!g_srcImage.data)
    {
        printf("读取图片错误，请确定目录下是否有imread函数指定的图片存在~！ \n");
        return false;
    }

    cv::cvtColor(g_srcImage, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(15, 150, 150), cv::Scalar(40, 255, 255), mask);
    cv::imshow("mask", mask);
    g_srcImage.copyTo(img_mask, mask);
    cv::imshow("img_mask", img_mask);

    cv::cvtColor(img_mask, g_srcImage, CV_HSV2BGR);

    // 转成灰度并模糊化降噪
    cvtColor(g_srcImage, g_grayImage, COLOR_BGR2GRAY);
    blur(g_grayImage, g_grayImage, Size(3, 3));

    // 创建窗口
    namedWindow(WINDOW_NAME1, WINDOW_AUTOSIZE);
    imshow(WINDOW_NAME1, g_srcImage);

    //创建滚动条并初始化
    createTrackbar("canny阈值", WINDOW_NAME1, &g_nThresh, g_nThresh_max, on_ThreshChange);
    on_ThreshChange(0, 0);

    waitKey(0);
    return (0);
}

//-----------------------------------【on_ThreshChange( )函数】------------------------------
//      描述：回调函数
//----------------------------------------------------------------------------------------------
void on_ThreshChange(int, void *)
{

    // 用Canny算子检测边缘
    Canny(g_grayImage, g_cannyMat_output, g_nThresh, g_nThresh * 1.5, 3);

    // 寻找轮廓
    findContours(g_cannyMat_output, g_vContours, g_vHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    std::cout << g_vContours.size() << std::endl;

    //面积最大的在前面
    sort(g_vContours.begin(), g_vContours.end(), Contour_Area);
    //存储前三个最大的多边形
    vector<vector<Point>> contours_poly(g_vContours.size());
    for (int i = 0; i < g_vContours.size(); i++)
    {
        double area = contourArea(g_vContours[i]);
        cout << area << endl;
        //逼近多边形曲线
        approxPolyDP(Mat(g_vContours[i]), contours_poly[i], 7, true);
    }
    cv::Mat Contour_0 = cv::Mat::zeros(g_srcImage.size(), CV_8UC1); //绘制边界矩形
    drawContours(Contour_0, contours_poly, 0, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
    cv::imshow("轮廓_0", Contour_0);
    cv::imwrite("/home/lzq/车门项目/Picture/轮廓0.png", Contour_0);

    cv::Mat Contour_1 = cv::Mat::zeros(g_srcImage.size(), CV_8UC1); //绘制边界矩形
    drawContours(Contour_1, contours_poly, 2, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
    cv::imshow("轮廓_1", Contour_1);
    cv::imwrite("/home/lzq/车门项目/Picture/轮廓1.png", Contour_1);

    cv::Mat Contour_2 = cv::Mat::zeros(g_srcImage.size(), CV_8UC1); //绘制边界矩形
    drawContours(Contour_2, contours_poly, 4, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
    cv::imshow("轮廓_2", Contour_2);
    cv::imwrite("/home/lzq/车门项目/Picture/轮廓2.png", Contour_2);

    // 绘出轮廓
    Mat drawing = Mat::zeros(g_cannyMat_output.size(), CV_8UC3);
    for (int i = 0; i < g_vContours.size(); i++)
    {
        // Scalar color = Scalar( g_rng.uniform(0, 255), g_rng.uniform(0,255), g_rng.uniform(0,255) );//任意值
        Scalar color = Scalar(255, 182, 193);
        drawContours(drawing, g_vContours, i, color, 2, 8, g_vHierarchy, 0, Point());
    }

    vector<Point> tempPoint; // 点集
    // 将所有点集存储到tempPoint
    for (int k = 0; k < g_vContours.size(); k++)
    {
        for (int m = 0; m < g_vContours[k].size(); m++)
        {
            tempPoint.push_back(g_vContours[k][m]);
        }
    }
    //对给定的 2D 点集，寻找最小面积的包围矩形
    RotatedRect box = minAreaRect(Mat(tempPoint));
    Point2f vertex[4];
    box.points(vertex);

    //绘制出最小面积的包围矩形
    for (int i = 0; i < 4; i++)
    {
        line(drawing, vertex[i], vertex[(i + 1) % 4], Scalar(100, 200, 211), 2, LINE_AA);
    }

    imshow(WINDOW_NAME2, drawing);
}

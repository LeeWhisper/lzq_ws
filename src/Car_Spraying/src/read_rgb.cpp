#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/ros.h"
#include <iostream>
using namespace cv;
using namespace std;
#define WINDOW_NAME "img"

Mat inputImage;
Mat outputImage;
Mat gray_image;
int GetPixelHSV(Mat src);
void on_MouseHandle(int event, int x, int y, int flags, void* param);

int GetPixelHSV(Mat srcHSV)
{
	inputImage = srcHSV.clone();
    // resize(inputImage,inputImage,Size(1280,1280),0,0);
	cvtColor(inputImage,gray_image,COLOR_BGR2GRAY);
	//设置鼠标操作回调函数
		namedWindow(WINDOW_NAME, 0);
	setMouseCallback(WINDOW_NAME, on_MouseHandle, &inputImage);

	while (1)
	{
		imshow(WINDOW_NAME, inputImage);
		waitKey(40);
	}
	return 0;
}

void on_MouseHandle(int event, int x, int y, int flags, void* param)
{
	Mat rgb, hsv,gray;
	rgb = *(Mat*)param;
	Mat temp;
	cvtColor(*(Mat*)param, hsv, COLOR_BGR2HSV);
	cvtColor(*(Mat*)param, gray, COLOR_BGR2GRAY);
	Point p(x, y);
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
	{

		// printf("b=%d\t", rgb.at<Vec3b>(p)[0]);
		// printf("g=%d\t", rgb.at<Vec3b>(p)[1]);
		// printf("r=%d\n", rgb.at<Vec3b>(p)[2]);

		printf("H=%d\t", hsv.at<Vec3b>(p)[0]);
		printf("S=%d\t", hsv.at<Vec3b>(p)[1]);
		printf("V=%d\n", hsv.at<Vec3b>(p)[2]);
		printf("x:%d,y:%d\n",p.x,p.y);
		// printf("G=%d\n", gray.at<uchar>(p));
		// circle(rgb, p, 2, Scalar(255), 10);
	}
	break;

	}

}

int main(){
GetPixelHSV(imread("/home/lzq/车门项目/Picture/output_1.png"));
return 0;
}

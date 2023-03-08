// MySAD.cpp : 定义控制台应用程序的入口点。
//
// #include "stdafx.h"

#include "camera_split/SAD.h"
#include "string.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc_c.h"

int main(int argc, char *argv[])
{
    Mat Img_L = imread("/home/lzq/lzq_ws/src/camera_split/left_img_rectified/left000.jpg", 0);
    Mat Img_R = imread("/home/lzq/lzq_ws/src/camera_split/right_img_rectified/right000.jpg", 0);
    Mat Disparity; //视差图

    // SAD mySAD;
    SAD mySAD(7, 30);
    Disparity = mySAD.computerSAD(Img_L, Img_R);

    imshow("Img_L", Img_L);
    imshow("Img_R", Img_R);
    imshow("Disparity", Disparity);

    waitKey();
    return 0;
}

// void BM()
// {
//     Mat img1 = imread("left.png", 0);
//     Mat img2 = imread("right.png", 0);
//     // CvStereoBMState *BMState = cvCreateStereoBMState();
//     cv::Ptr<cv::StereoBM> BMState = cv::StereoBM::create(16, 9);
//     assert(BMState);
//     BMState->setPreFilterSize(9);
//     BMState->stPreFilterCap = 31;
//     BMState->SADWindowSize = 15;
//     BMState->minDisparity = 0;
//     BMState->numberOfDisparities = 64;
//     BMState->textureThreshold = 10;
//     BMState->uniquenessRatio = 15;
//     BMState->speckleWindowSize = 100;
//     BMState->speckleRange = 32;
//     BMState->disp12MaxDiff = 1;

//     CvMat *disp = cvCreateMat(img1->height, img1->width, CV_16S);
//     CvMat *vdisp = cvCreateMat(img1->height, img1->width, CV_8U);
//     int64 t = getTickCount();
//     cvFindStereoCorrespondenceBM(img1, img2, disp, BMState);
//     t = getTickCount() - t;
//     cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
//     cvSave("disp.xml", disp);
//     cvNormalize(disp, vdisp, 0, 255, CV_MINMAX);
//     cvNamedWindow("BM_disparity", 0);
//     cvShowImage("BM_disparity", vdisp);
//     cvWaitKey(0);
//     // cvSaveImage("cones\\BM_disparity.png",vdisp);
//     cvReleaseMat(&disp);
//     cvReleaseMat(&vdisp);
//     cvDestroyWindow("BM_disparity");
// }

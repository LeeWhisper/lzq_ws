#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/imgproc/types_c.h>
#include "ros/ros.h"

using namespace std;
using namespace cv;

const int imageWidth = 640; //摄像头的分辨率
const int imageHeight = 480;
Size imageSize = Size(imageWidth, imageHeight);

Mat grayImageL, grayImageR;
Mat rectifyImageL, rectifyImageR;

Rect validROIL; //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域, 其内部的所有像素都有效
Rect validROIR;

Mat mapLx, mapLy, mapRx, mapRy; //映射表
Mat Rl, Rr, Pl, Pr, Q;          //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
Mat xyz;                        //三维坐标

Point origin;              //鼠标按下的起始点
Rect selection;            //定义矩形选框
bool selectObject = false; //是否选择对象

int numberOfDisparities = ((imageSize.width / 8) + 15) & -16;
int numDisparities = 6;

cv::Ptr<cv::StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

Mat cameraMatrixL = (Mat_<double>(3, 3) << 4.2562214518886043e+02, 0., 3.1275413089548573e+02, 0.,
                     4.2504704931897089e+02, 2.5068093587812987e+02, 0., 0., 1.);
Mat distCoeffL = (Mat_<double>(5, 1) << 1.6788713522051915e-01, -2.2813225785432234e-01,
                  -8.9089453911466940e-05, -8.0081043818621832e-04,
                  1.2579331154709636e-01);

Mat cameraMatrixR = (Mat_<double>(3, 3) << 4.2531835808591370e+02, 0., 3.3002757529428715e+02, 0.,
                     4.2456726376806893e+02, 2.3098157730970146e+02, 0., 0., 1.);
Mat distCoeffR = (Mat_<double>(5, 1) << 1.6866144814487635e-01, -2.4567047646458187e-01,
                  -9.1619189048586496e-04, -9.6450000750443606e-04,
                  1.3896731032668574e-01);

//左右目之间的R,t可通过stereoCalibrate()或matlab工具箱calib求得
Mat T = (Mat_<double>(3, 1) << -6.1352666379150818e+01, 3.0678685341246570e-02,
         1.1539613423182472e-01);                            // T平移向量
Mat rec = (Mat_<double>(3, 1) << 0.00468, 0.02159, 0.00015); // rec旋转向量
Mat R;                                                       // R 旋转矩阵
Mat frame, f1, f2;
Mat disp, disp8;

/*****立体匹配*****/
void stereo_match(int, void *)
{
    sgbm->setPreFilterCap(32);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = rectifyImageL.channels();
    sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);
    sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    sgbm->compute(rectifyImageL, rectifyImageR, disp); //输入图像必须为灰度图

    disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16) * 16.)); //计算出的视差是CV_16S格式
    reprojectImageTo3D(disp, xyz, Q, true);                                 //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);
}

/*****描述：鼠标操作回调*****/
static void onMouse(int event, int x, int y, int, void *)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
    }

    switch (event)
    {
    case EVENT_LBUTTONDOWN: //鼠标左按钮按下的事件
        origin = Point(x, y);
        selection = Rect(x, y, 0, 0);
        selectObject = true;
        cout << "point: " << origin << endl;
        cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
        cout << xyz.at<Vec3f>(origin)[0] << xyz.at<Vec3f>(origin)[1] << xyz.at<Vec3f>(origin)[2] << endl;

        break;
    case EVENT_LBUTTONUP: //鼠标左按钮释放的事件
        selectObject = false;
        if (selection.width > 0 && selection.height > 0)
            break;
    }
}

/*****主函数*****/
int main(int argc, char  *argv[])
{
    Rodrigues(rec, R); // Rodrigues变换
    //经过双目标定得到摄像头的各项参数后，采用OpenCV中的stereoRectify(立体校正)得到校正旋转矩阵R、投影矩阵P、重投影矩阵Q
    // flags-可选的标志有两种零或者 CV_CALIB_ZERO_DISPARITY ,如果设置 CV_CALIB_ZERO_DISPARITY 的话，该函数会让两幅校正后的图像的主点有相同的像素坐标。否则该函数会水平或垂直的移动图像，以使得其有用的范围最大
    // alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);
    //再采用映射变换计算函数initUndistortRectifyMap得出校准映射参数,该函数功能是计算畸变矫正和立体校正的映射变换
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

    VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_HEIGHT, 376);
    cap.set(CAP_PROP_FRAME_WIDTH, 1344);
    namedWindow("disparity", WINDOW_AUTOSIZE);
    namedWindow("paramemnt", WINDOW_NORMAL);
    createTrackbar("numDisparities:\n", "paramemnt", &numDisparities, 20, stereo_match);
    setMouseCallback("disparity", onMouse, 0);

    while (1)
    {
        cap >> frame;
        imshow("video", frame);
        f1 = frame.colRange(0, 640);
        f2 = frame.colRange(640, 1280);

        cvtColor(f1, grayImageL, CV_BGR2GRAY);
        cvtColor(f2, grayImageR, CV_BGR2GRAY);
        //然后用remap来校准输入的左右图像
        // interpolation-插值方法，但是不支持最近邻插值
        remap(grayImageL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
        remap(grayImageR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
        // imshow("img_L", rectifyImageL);
        // imshow("img_R", rectifyImageR);

        stereo_match(0, 0);
        waitKey(1);
    }
    waitKey(0);
    return 0;
}
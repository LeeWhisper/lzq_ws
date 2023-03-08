#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Int32.h"
#include "cv_bridge/cv_bridge.h"
#include "boost/format.hpp"
#include "camera_split/SAD.h"

using namespace std;
using namespace cv;
using namespace cv::dnn;

boost::format left_imgs("/home/lzq/lzq_ws/src/camera_split/left_img_rectified/left%03d.jpg");
boost::format right_imgs("/home/lzq/lzq_ws/src/camera_split/right_img_rectified/right%03d.jpg");

class capture
{
private:
    String modelConfiguration = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov3-tiny.cfg";
    String modelBinary = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov3-tiny.weights";
    string namesfile = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/coco.names";
    ros::NodeHandle nh;
    ros::Subscriber sub_key;
    image_transport::Subscriber sub_img;

    const int imageWidth = 640; //摄像头的分辨率
    const int imageHeight = 480;
    Size imageSize = Size(imageWidth, imageHeight);

    Mat mapLx, mapLy, mapRx, mapRy;

    Mat cameraMatrixL = (Mat_<double>(3, 3) << 4.2562214518886043e+02, 0., 3.1275413089548573e+02, 0.,
                         4.2504704931897089e+02, 2.5068093587812987e+02, 0., 0., 1.);
    Mat distCoeffL = (Mat_<double>(5, 1) << 1.6788713522051915e-01, -2.2813225785432234e-01,
                      -8.9089453911466940e-05, -8.0081043818621832e-04,
                      1.2579331154709636e-01);
    Mat Rl = (Mat_<double>(3, 3) << 9.9999379447559955e-01, -1.1738767337808789e-03,
              -3.3215995704182961e-03, 1.1707687781728055e-03,
              9.9999887522444708e-01, -9.3747016413628869e-04,
              3.3226963087785804e-03, 9.3357552157168479e-04,
              9.9999404404525594e-01);
    Mat Pl = (Mat_<double>(3, 4) << 4.2480715654351991e+02, 0., 3.2229185485839844e+02, 0., 0.,
              4.2480715654351991e+02, 2.4044572257995605e+02, 0., 0., 0., 1.,
              0.);

    Mat cameraMatrixR = (Mat_<double>(3, 3) << 4.2531835808591370e+02, 0., 3.3002757529428715e+02, 0.,
                         4.2456726376806893e+02, 2.3098157730970146e+02, 0., 0., 1.);
    Mat distCoeffR = (Mat_<double>(5, 1) << 1.6866144814487635e-01, -2.4567047646458187e-01,
                      -9.1619189048586496e-04, -9.6450000750443606e-04,
                      1.3896731032668574e-01);
    Mat Rr = (Mat_<double>(3, 3) << 9.9999810615806084e-01, -5.0003739121021477e-04,
              -1.8808622754005974e-03, 5.0179676557800300e-04,
              9.9999943693794568e-01, 9.3505283139414398e-04,
              1.8803936549799509e-03, -9.3599487115815830e-04,
              9.9999779401421862e-01);
    Mat Pr = (Mat_<double>(3, 4) << 4.2480715654351991e+02, 0., 3.2229185485839844e+02,
              -2.6063101110284220e+04, 0., 4.2480715654351991e+02,
              2.4044572257995605e+02, 0., 0., 0., 1., 0.);

    float al, ar, d, f, xl, xr, z;

    void img_callback(const sensor_msgs::ImageConstPtr &img);
    void flag_callback(std_msgs::Int32 flag);

public:
    int xLeftBottom;
    int yLeftBottom;
    int xRightTop;
    int yRightTop;
    //置信度阈值
    float confidenceThreshold = 0.25;
    //是否找到目标
    bool get_flag = false, avoid_flag;
    vector<string> classNamesVec;
    dnn::Net net;

    cv_bridge::CvImagePtr color_ptr;
    int key;
    int count = 0;
    capture();
};

capture::capture()
{
    d = 60;
    f = 2.8;
    al = 4.2562214518886043e+02 / f;
    ar = 4.2531835808591370e+02 / f;

    image_transport::ImageTransport it(nh);
    sub_key = nh.subscribe("/key_board", 10, &capture::flag_callback, this);
    sub_img = it.subscribe("/usb_cam/image_raw", 1, &capture::img_callback, this);
    net = readNetFromDarknet(modelConfiguration, modelBinary);

    ifstream classNamesFile(namesfile);
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }
}

void capture::flag_callback(std_msgs::Int32 flag)
{
    key = flag.data;
    cout << "key: " << key << endl;
}

void capture::img_callback(const sensor_msgs::ImageConstPtr &img)
{

    Mat frame;
    color_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    frame = color_ptr->image;
    if (frame.empty())
        if (frame.channels() == 4)
            cvtColor(frame, frame, COLOR_BGRA2BGR);

    // imshow("capture", frame);
    waitKey(30);

    double fScale = 1; //定义缩放系数，对2560*720图像进行缩放显示（2560*720图像过大，液晶屏分辨率较小时，需要缩放才可完整显示在屏幕）
    Size dsize = Size(frame.cols * fScale, frame.rows * fScale);
    Mat imagedst = Mat(dsize, CV_8UC3);
    resize(frame, imagedst, dsize);
    Mat frame_L = frame(Rect(0, 0, int(frame.cols / 2), frame.rows));                   //获取缩放后左Camera的图像
    Mat frame_R = frame(Rect(int(frame.cols / 2), 0, int(frame.cols / 2), frame.rows)); //获取缩放后右Camera的图像

    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

    remap(frame_L, frame_L, mapLx, mapLy, INTER_LINEAR);
    remap(frame_R, frame_R, mapRx, mapRy, INTER_LINEAR);

    // imshow("frame_L", rectifyImageL2);
    // imshow("frame_R", rectifyImageR2);
    Mat canvas;
    double sf;
    int w = 640, h = 480;
    // sf = 600. / MAX(imageSize.width, imageSize.height);
    // w = cvRound(imageSize.width * sf);
    // h = cvRound(imageSize.height * sf);
    canvas.create(h, w * 2, CV_8UC3);

    /*左图像画到画布上*/
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                    //得到画布的一部分
    resize(frame_L, canvasPart, canvasPart.size(), 0, 0, INTER_AREA); //把图像缩放到跟canvasPart一样大小

    /*右图像画到画布上*/
    canvasPart = canvas(Rect(w, 0, w, h)); //获得画布的另一部分
    resize(frame_R, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);

    /****************************************************************************************************/
    // 图像预处理
    // blobFromImage:在进行深度学习或者学习分类时，用来对图片预处理，
    // 先整体像素值减去平均值（mean），再通过缩放系数（scalarfactor）对像素值进行缩放
    Mat inputBlob = blobFromImage(frame_L, 1 / 255.F, Size(416, 416), Scalar(), true, false);
    // 设置网络输入
    net.setInput(inputBlob, "data");
    // 创建预测矩阵，存储网络预测的返回值
    Mat detectionMat = net.forward(); // 前向预测
    vector<double> layersTimings;
    // 计算帧率以及处理所需时间
    double freq = getTickFrequency() / 1000;                // 返回CPU频率，次/s，除以1000得到次/ms
    double time = net.getPerfProfile(layersTimings) / freq; // getPerfProfile获取总次数，时间=总次数/频率
    ostringstream ss;
    ss << "FPS: " << 1000 / time << ";time: " << time << "ms";
    putText(canvas, ss.str(), Point(20, 20), 0, 0.5, Scalar(0, 0, 255));
    for (int i = 0; i < detectionMat.rows; i++)
    {
        // 设定预测种类数
        const int probability_index = 5;
        const int probability_size = detectionMat.cols - probability_index;
        float *prob_array_ptr = &detectionMat.at<float>(i, probability_index);
        // 由偏移量计算种类
        size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
        float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);
        // 判断置信度是否达到阈值
        if (confidence > confidenceThreshold)
        {
            //只显示人物的识别框

            // 获取框的中心坐标以及框的长和宽
            float x = detectionMat.at<float>(i, 0);
            float y = detectionMat.at<float>(i, 1);
            float width = detectionMat.at<float>(i, 2);
            float height = detectionMat.at<float>(i, 3);
            // cout << "x_L = " << x * frame_L.cols << endl;
            // cout << "y_L = " << y * frame_L.rows << endl;
            xl = (x * frame_L.cols - 3.1275413089548573e+02) / al;

            // 计算相对于图像的坐标，也就是最终在屏幕上看到的框的坐标
            xLeftBottom = static_cast<int>((x - width / 2) * frame_L.cols);
            yLeftBottom = static_cast<int>((y - height / 2) * frame_L.rows);
            xRightTop = static_cast<int>((x + width / 2) * frame_L.cols);
            yRightTop = static_cast<int>((y + height / 2) * frame_L.rows);

            // 绘制框
            Rect object(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);
            // circle(frame, Point((xLeftBottom+2*(xRightTop - xLeftBottom),yLeftBottom+2*yRightTop - yLeftBottom)), 1, (0,0,255), 5);
            rectangle(canvas, object, Scalar(0, 255, 0), 2);

            // 显示类别名称
            if (objectClass < classNamesVec.size())
            {
                ss.str("");
                ss << confidence;
                confidence = 0;
                String conf(ss.str());
                String label;

                //只显示人物的标签

                label = String(classNamesVec[objectClass]) + ": " + conf;
                int baseLine = 0;
                Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                rectangle(canvas, Rect(Point(xLeftBottom, yLeftBottom), Size(labelSize.width, labelSize.height + baseLine)),
                          Scalar(255, 255, 255), CV_FILLED);
                putText(canvas, String(label), Point(xLeftBottom, yLeftBottom + labelSize.height),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

                String pre_name = String(classNamesVec[objectClass]);
                String real_name = "person";
                if (pre_name == real_name)
                {
                    // cout << "[track]找到人物" << endl;
                    get_flag = true;
                }
            }
        }
    }

    // 先整体像素值减去平均值（mean），再通过缩放系数（scalarfactor）对像素值进行缩放
    Mat inputBlob_1 = blobFromImage(frame_R, 1 / 255.F, Size(416, 416), Scalar(), true, false);
    // 设置网络输入
    net.setInput(inputBlob_1, "data");
    // 创建预测矩阵，存储网络预测的返回值
    Mat detectionMat_1 = net.forward(); // 前向预测

    for (int i = 0; i < detectionMat_1.rows; i++)
    {
        // 设定预测种类数
        const int probability_index = 5;
        const int probability_size = detectionMat_1.cols - probability_index;
        float *prob_array_ptr = &detectionMat_1.at<float>(i, probability_index);
        // 由偏移量计算种类
        size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
        float confidence = detectionMat_1.at<float>(i, (int)objectClass + probability_index);
        // 判断置信度是否达到阈值
        if (confidence > confidenceThreshold)
        {
            //只显示人物的识别框

            // 获取框的中心坐标以及框的长和宽
            float x = detectionMat_1.at<float>(i, 0);
            float y = detectionMat_1.at<float>(i, 1);
            float width = detectionMat_1.at<float>(i, 2);
            float height = detectionMat_1.at<float>(i, 3);
            xr = (x * frame_R.cols - 3.3002757529428715e+02) / ar;
            // cout << "x_R = " << x * frame_R.cols << endl;
            // cout << "y_R = " << y * frame_R.rows << endl;

            // 计算相对于图像的坐标，也就是最终在屏幕上看到的框的坐标
            xLeftBottom = static_cast<int>((x - width / 2) * frame_R.cols + frame_L.cols);
            yLeftBottom = static_cast<int>((y - height / 2) * frame_R.rows);
            xRightTop = static_cast<int>((x + width / 2) * frame_R.cols + frame_L.cols);
            yRightTop = static_cast<int>((y + height / 2) * frame_R.rows);

            // 绘制框
            Rect object(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);
            // circle(frame, Point((xLeftBottom+2*(xRightTop - xLeftBottom),yLeftBottom+2*yRightTop - yLeftBottom)), 1, (0,0,255), 5);
            rectangle(canvas, object, Scalar(0, 0, 255), 2);

            // 显示类别名称
            if (objectClass < classNamesVec.size())
            {
                ss.str("");
                ss << confidence;
                confidence = 0;
                String conf(ss.str());
                String label;

                //只显示人物的标签

                label = String(classNamesVec[objectClass]) + ": " + conf;
                int baseLine = 0;
                Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                rectangle(canvas, Rect(Point(xLeftBottom, yLeftBottom), Size(labelSize.width, labelSize.height + baseLine)),
                          Scalar(255, 255, 255), CV_FILLED);
                putText(canvas, String(label), Point(xLeftBottom, yLeftBottom + labelSize.height),
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

                String pre_name = String(classNamesVec[objectClass]);
                String real_name = "person";
                if (pre_name == real_name)
                {
                    // cout << "[track]找到人物" << endl;
                    get_flag = true;
                }
            }
        }
    }

    imshow("rectified", canvas);

    // Mat Disparity; //视差图
    // // SAD mySAD;
    // SAD mySAD(7, 30);
    // Disparity = mySAD.computerSAD(frame_L, frame_R);
    // imshow("Disparity", Disparity);


    // 视差测距
    z = d * f / (xl - xr);
    cout << "z = " << z << endl;
    
    // imshow("YOLOv2:frame", frame);

    if (key == 32)
    {
        imwrite((left_imgs % count).str(), frame_L);
        imwrite((right_imgs % count).str(), frame_R);
        cout << "123" << endl;
        count++;
        key--;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "capture");
    capture cap;
    ros::spin();

    return 0;
}

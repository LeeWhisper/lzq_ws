#include<iostream>
#include<string>
#include<opencv2/opencv.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace cv;

boost::format left_imgs("./1_left/left%03d.jpg");
boost::format right_imgs("./2_right/right%03d.jpg");

int main(int argc, char* argv[]) //程序主函数
{
    string command;
    if (access("left", 0) == -1){//如果文件夹不存在
        //则创建
        command = "mkdir -p left";
        system(command.c_str());
    }
    if (access("right", 0) == -1){//如果文件夹不存在
        //则创建
        command = "mkdir -p right";
        system(command.c_str());
    }

    VideoCapture cap(2);//打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1，主要是在设备管理器中查看自己摄像头的编号。

    //--------------------------------------------------------------------------------------

    cap.set(CAP_PROP_FRAME_WIDTH, 2560); //设置捕获视频的宽度
    cap.set(CAP_PROP_FRAME_HEIGHT, 720); //设置捕获视频的高度

    if (!cap.isOpened())             //判断是否成功打开相机
    {
        cout << "摄像头打开失败!" << endl;
        return -1;
    }
    Mat frame, frame_L,frame_R;

    cap >> frame;                //从相机捕获一帧图像

    Mat grayImage;                //用于存放灰度数据

    double fScale = 1;             //定义缩放系数，对2560*720图像进行缩放显示（2560*720图像过大，液晶屏分辨率较小时，需要缩放才可完整显示在屏幕）
    Size dsize = Size(frame.cols*fScale, frame.rows*fScale);
    Mat imagedst = Mat(dsize, CV_32S);
    resize(frame, imagedst, dsize);
    
    
    char key;
    int count = 0;

    while (true)
    {
        key = waitKey(50);
         cap >> frame;   //从相机捕获一帧图像
        resize(frame, imagedst, dsize);     //对捕捉的图像进行缩放操作
        imshow("Stereo Video", frame);

        frame_L = imagedst(Rect(0, 0, int(frame.cols/2), frame.rows)); //获取缩放后左Camera的图像
        frame_R = imagedst(Rect(int(frame.cols/2), 0, int(frame.cols/2), frame.rows)); //获取缩放后右Camera的图像

        if (key == 27) //按下ESC退出
            break;
        if (key == 32) // 按下空格开始拍照图片保存在工程文件下
        {
            imwrite((left_imgs % count).str(), frame_L);
            imwrite((right_imgs % count).str(), frame_R);

            count++;
//            imshow("图片left", frame_L);
//            imshow("图片right", frame_R);
        }
    }

    return 0;
}

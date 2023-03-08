#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>//DNN模块
#include "opencv2/core/cuda_types.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/cuda.inl.hpp"
#include <librealsense2/rs.hpp> 
//#include "opencv2/contrib/contrib.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
using namespace std;
using namespace cv;
using namespace cv::dnn;//使用cv和DNN命名空间
//置信度阈值
float confidenceThreshold = 0.25;
void video_detection();
void image_detection();
int main(int argc, char** argv)
{
	//选择预测图片还是视频流
	//image_detection();
	
	video_detection();
	
}

void video_detection() {
	
	//先设置训练好的网络模型以及参数文件路径，建议选择相对路径，我设置没成功，就用了绝对路径
	//也可以加载修改后的网络，只需要将对应的网络结构以及训练好的网络模型加载进来就行了
	// String modelConfiguration = "/home/nvidia/cmake_test/yolo_cv/yolov3/yolov3.cfg";
	String modelConfiguration = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov2-tiny-voc.cfg";
	// String modelBinary = "/home/nvidia/cmake_test/yolo_cv/yolov3/yolov3.weights";
	String modelBinary = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov2-tiny-voc.weights";
	//读取网络
	dnn::Net net = readNetFromDarknet(modelConfiguration, modelBinary);
	//net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);//opencv4.4以上版本可以使用cuda进行加速
	//net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);//4.4版本以下需要将这两句代码删掉
	if (net.empty())//判断网络是否加载正常
	{
		printf("Could not load net...\n");
		return;
	}
	//加载预测目标的名称
	vector<string> classNamesVec;//
	ifstream classNamesFile("/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/voc.names");
	if (classNamesFile.is_open())
	{
		string className = "";
		while (std::getline(classNamesFile, className))
			classNamesVec.push_back(className);
	}
	//开启摄像头，或直接在函数中传入一段视频
	// VideoCapture capture(0); 
	VideoCapture capture;
	capture.open(0);
	//capture.open("D:/vcprojects/images/fbb.avi");
	if (!capture.isOpened()) {
		printf("could not open the camera...\n");
		return;
	}
	//D435
	// rs2::pipeline pipe;
	// pipe.start();

	//若想移植到ROS下，只需要建立一个能够接收图像的节点，再把for循环的内容可以放到图像回调函数中即可
	for(;;)
	{
		// rs2::frameset data = pipe.wait_for_frames();
		// rs2::frame color = data.get_color_frame();

		// const int w = color.as<rs2::video_frame>().get_width();
		// const int h = color.as<rs2::video_frame>().get_height();

		// Mat frame(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

		Mat frame;
		capture>>frame;
		if (frame.empty())
			if (frame.channels() == 4)
				cvtColor(frame, frame, COLOR_BGRA2BGR);//转换通道，opencv与YOLO所使用通道顺序不同
		//图像预处理	
		Mat inputBlob = blobFromImage(frame, 1 / 255.F, Size(416, 416), Scalar(), true, false);
		//设置网络输入
		net.setInput(inputBlob, "data");
		//创建预测矩阵，存储网络预测的返回值
		Mat detectionMat = net.forward();//前向预测
		vector<double> layersTimings;
		//计算帧率以及处理所需时间
		double freq = getTickFrequency() / 1000;
		double time = net.getPerfProfile(layersTimings) / freq;
		ostringstream ss;
		ss << "FPS: " << 1000 / time << " ; time: " << time << " ms";
		putText(frame, ss.str(), Point(20, 20), 0, 0.5, Scalar(0, 0, 255));
		//
		for (int i = 0; i < detectionMat.rows; i++)
		{
			//预测五个种类
			const int probability_index = 5;
			const int probability_size = detectionMat.cols - probability_index;
			float *prob_array_ptr = &detectionMat.at<float>(i, probability_index);
			//由偏移量计算种类
			size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
			float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);
			//判断置信度是否达到阈值，移植到ROS时可以把框的坐标以及概率和置信度等数据打包成自己想要的数据类型。
			if (confidence > confidenceThreshold)
			{
				//获取框的左上角坐标以及框的长和宽
				float x = detectionMat.at<float>(i, 0);
				float y = detectionMat.at<float>(i, 1);
				float width = detectionMat.at<float>(i, 2);
				float height = detectionMat.at<float>(i, 3);
				//计算相对于图像的坐标，也就是最终在屏幕上看到的框的坐标
				int xLeftBottom = static_cast<int>((x - width / 2) * frame.cols);
				int yLeftBottom = static_cast<int>((y - height / 2) * frame.rows);
				int xRightTop = static_cast<int>((x + width / 2) * frame.cols);
				int yRightTop = static_cast<int>((y + height / 2) * frame.rows);
				//画框
				Rect object(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);

                cout << x << endl;
                cout << y << endl;
                cout << width << endl;
                cout << height << endl;

				rectangle(frame, object, Scalar(0, 255, 0));

				//显示类别
				if (objectClass < classNamesVec.size())
				{
					ss.str("");
					ss << confidence;
					String conf(ss.str());
					String label = String(classNamesVec[objectClass]) + ": " + conf;
					int baseLine = 0;
					Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
					rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom),
						Size(labelSize.width, labelSize.height + baseLine)),
						Scalar(255, 255, 255), CV_FILLED);
					putText(frame, label, Point(xLeftBottom, yLeftBottom + labelSize.height),
						FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
				}
			}
		}
		
		imshow("YOLOv3: Detections", frame);
		waitKey(30) ;
	}
}
//过程与以上相似，这个函数用来预测图片。
void image_detection() {
	String modelConfiguration = "/home/dreamtale/cmake_practice/cmake/yolo_cv/yolov2-tiny-voc/yolov2-tiny-voc.cfg";
	String modelBinary = "/home/dreamtale/cmake_practice/cmake/yolo_cv/yolov2-tiny-voc/yolov2-tiny-voc.weights";
	dnn::Net net = readNetFromDarknet(modelConfiguration, modelBinary);
	if (net.empty())
	{
		printf("Could not load net...\n");
		return;
	}
	vector<string> classNamesVec;
	ifstream classNamesFile("/home/dreamtale/cmake_practice/cmake/yolo_cv/yolov2-tiny-voc/voc.names");
	if (classNamesFile.is_open())
	{
		string className = "";
		while (std::getline(classNamesFile, className))
			classNamesVec.push_back(className);
	}

	// ����ͼ��
	Mat frame = imread("/home/dreamtale/cmake_practice/cmake/yolo_cv/xiaomaolu.jpg");
	Mat inputBlob = blobFromImage(frame, 1 / 255.F, Size(416, 416), Scalar(), true, false);
	net.setInput(inputBlob, "data");

	// ���
	Mat detectionMat = net.forward("detection_out");
	vector<double> layersTimings;
	double freq = getTickFrequency() / 1000;
	double time = net.getPerfProfile(layersTimings) / freq;
	ostringstream ss;
	ss << "detection time: " << time << " ms";
	putText(frame, ss.str(), Point(20, 20), 0, 0.5, Scalar(0, 0, 255));

	// ������
	for (int i = 0; i < detectionMat.rows; i++)
	{
		const int probability_index = 5;
		const int probability_size = detectionMat.cols - probability_index;
		float *prob_array_ptr = &detectionMat.at<float>(i, probability_index);
		size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
		float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);
		if (confidence > confidenceThreshold)
		{
			float x = detectionMat.at<float>(i, 0);
			float y = detectionMat.at<float>(i, 1);
			float width = detectionMat.at<float>(i, 2);
			float height = detectionMat.at<float>(i, 3);
			int xLeftBottom = static_cast<int>((x - width / 2) * frame.cols);
			int yLeftBottom = static_cast<int>((y - height / 2) * frame.rows);
			int xRightTop = static_cast<int>((x + width / 2) * frame.cols);
			int yRightTop = static_cast<int>((y + height / 2) * frame.rows);
			Rect object(xLeftBottom, yLeftBottom,
				xRightTop - xLeftBottom,
				yRightTop - yLeftBottom);
			rectangle(frame, object, Scalar(0, 0, 255), 2, 8);
			if (objectClass < classNamesVec.size())
			{
				ss.str("");
				ss << confidence;
				String conf(ss.str());
				String label = String(classNamesVec[objectClass]) + ": " + conf;
				int baseLine = 0;
				Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
				rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom),
					Size(labelSize.width, labelSize.height + baseLine)),
					Scalar(255, 255, 255), CV_FILLED);
				putText(frame, label, Point(xLeftBottom, yLeftBottom + labelSize.height),
					FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
			}
		}
	}
	imshow("YOLO-Detections", frame);
	waitKey(0);
	return;
}

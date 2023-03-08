#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "track_avoid/Depth.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
using namespace cv::dnn;

class track
{
private:
    /* data */
    String modelConfiguration = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov2-tiny-voc.cfg";
    String modelBinary = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov2-tiny-voc.weights";
    string namesfile = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/voc.names";
    ros::NodeHandle nh;
    image_transport::Subscriber sub_image, sub_depth;
	ros::Publisher pub, pub_searching, pub_get_flag;
	ros::Subscriber sub_avoid_flag;

    void image_callback(const sensor_msgs::ImageConstPtr &image);
    void depth_callback(const sensor_msgs::ImageConstPtr &image_depth);
    void avoid_flag_callback(const std_msgs::Bool flag);
    
public:

    int xLeftBottom;
    int yLeftBottom;
    int xRightTop;
    int yRightTop;
	track_avoid::Depth dep_loc;

    //置信度阈值
    float confidenceThreshold = 0.25;
    //是否找到目标
    bool get_flag = false, avoid_flag;
	std_msgs::Bool get_flag_s;

    cv_bridge::CvImagePtr color_ptr, depth_ptr;
    cv::Mat color_img, depth_img;

    vector<string>classNamesVec;
    dnn::Net net;
    track();
};

track::track()
{
    image_transport::ImageTransport it(nh);
    sub_image = it.subscribe("/camera/color/image_raw", 1, &track::image_callback, this);
	sub_depth = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &track::depth_callback, this);
    sub_avoid_flag = nh.subscribe("/avoid_flag", 10, &track::avoid_flag_callback, this);

	pub = nh.advertise<track_avoid::Depth>("/track_person", 10);
	pub_searching = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	pub_get_flag = nh.advertise<std_msgs::Bool>("/get_flag", 1);

    net = readNetFromDarknet(modelConfiguration, modelBinary);
    ifstream classNamesFile(namesfile);
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className))
                classNamesVec.push_back(className);
    }   
}

void track::avoid_flag_callback(const std_msgs::Bool flag)
{
    if (flag.data)
        avoid_flag = true;
    else
        avoid_flag = false;
	// cout << "avoid_flag: " << avoid_flag << endl;
    
}

void track::image_callback(const sensor_msgs::ImageConstPtr &image)
{
    Mat frame;
    color_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	frame = color_ptr->image;
	
	// color_img = frame;
	if (frame.empty())
		if(frame.channels() == 4)
			cvtColor(frame, frame, COLOR_BGRA2BGR);
	/****************************************************************************************************/
	// 图像预处理
	// blobFromImage:在进行深度学习或者学习分类时，用来对图片预处理，
	// 先整体像素值减去平均值（mean），再通过缩放系数（scalarfactor）对像素值进行缩放
	Mat inputBlob = blobFromImage(frame, 1 / 255.F, Size(416, 416), Scalar(), true, false);
	// 设置网络输入
	net.setInput(inputBlob, "data");
	// 创建预测矩阵，存储网络预测的返回值
	Mat detectionMat = net.forward();	// 前向预测
	vector<double> layersTimings;
	// 计算帧率以及处理所需时间
	double freq = getTickFrequency() / 1000;				// 返回CPU频率，次/s，除以1000得到次/ms
	double time = net.getPerfProfile(layersTimings) / freq; // getPerfProfile获取总次数，时间=总次数/频率
	ostringstream ss;
	ss << "FPS: " << 1000 / time << ";time: " << time << "ms";
	putText(frame, ss.str(), Point(20,20), 0, 0.5, Scalar(0, 0, 255));
	for(int i = 0; i < detectionMat.rows; i++)
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
			if (String(classNamesVec[objectClass]) == "person")	
			{
				// 获取框的左上角坐标以及框的长和宽
				float x = detectionMat.at<float>(i, 0);
				float y = detectionMat.at<float>(i, 1);
				float width = detectionMat.at<float>(i, 2);
				float height = detectionMat.at<float>(i, 3);

				// 计算相对于图像的坐标，也就是最终在屏幕上看到的框的坐标
				xLeftBottom = static_cast<int>((x - width / 2) * frame.cols);
				yLeftBottom = static_cast<int>((y - height / 2) * frame.rows);
				xRightTop = static_cast<int>((x + width / 2) * frame.cols);
				yRightTop = static_cast<int>((y + height / 2) * frame.rows);

				// 绘制框
				Rect object(xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom);		
				// circle(frame, Point((xLeftBottom+2*(xRightTop - xLeftBottom),yLeftBottom+2*yRightTop - yLeftBottom)), 1, (0,0,255), 5);
				rectangle(frame, object, Scalar(0, 255, 0), 2);
			}
			// 显示类别名称
			if (objectClass < classNamesVec.size())
			{
				ss.str("");
				ss << confidence;
				confidence = 0;
				String conf(ss.str());
				String label;

				//只显示人物的标签
				if (String(classNamesVec[objectClass]) == "person")		
				{	
					label = String(classNamesVec[objectClass]) + ": " + conf;		
					int baseLine = 0;
					Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
					rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom),
								Size(labelSize.width, labelSize.height + baseLine)),
								Scalar(255, 255, 255), CV_FILLED);
					putText(frame, String(label), Point(xLeftBottom, yLeftBottom + labelSize.height),
							FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
				}
				String pre_name = String(classNamesVec[objectClass]);
				String real_name = "person";
				if (pre_name == real_name)
				{
					cout << "[track]找到人物" << endl;
					get_flag = true;
				}

			}
		}
	}	
	imshow("YOLOv2:Detections", frame);
	waitKey(30);
}

void get_roi_depth(int &depth, int &locate, int x, int y, int width, int height, cv::Mat roi)
{
	int x_step = int(width / 11);
	int y_step = int(height / 11);
	int x1 = x + 5*x_step;
	int y1 = y + 5*y_step;
	int x2 = x + 6*x_step;
	int y2 = y + 6*y_step;
	int x3 = int (x + 5.5*x_step);
	int y3 = int (y + 5.5*y_step);
	int sum = 0;
	int ave = 0;
	int count = 0;
	int feature_point[5];

	feature_point[0] = roi.at<uint16_t>(y1, x1);
	feature_point[1] = roi.at<uint16_t>(y1, x2);
	feature_point[2] = roi.at<uint16_t>(y2, x1);
	feature_point[3] = roi.at<uint16_t>(y2, x2);
	feature_point[4] = roi.at<uint16_t>(y3, x3);

	sort(feature_point, feature_point+5);
	depth = feature_point[2] / 10;
	locate = x3;

}

void track::depth_callback(const sensor_msgs::ImageConstPtr &image_depth)
{
	depth_ptr = cv_bridge::toCvCopy(image_depth, sensor_msgs::image_encodings::TYPE_16SC1);
	depth_img = depth_ptr -> image;
	geometry_msgs::Twist twist;
	cout << "avoid_flag: " << avoid_flag << endl;
	cout << "get_flag: " << get_flag << endl;
	if (get_flag)
	{
		get_flag_s.data = true;
	}				
	else
	{
		get_flag_s.data = false;
	}
	pub_get_flag.publish(get_flag_s);
	
	if (avoid_flag == false)
	{
		if (get_flag == true)
		{			
			get_flag = false;
			int depth = dep_loc.depth;
			int loction = dep_loc.locate;
			get_roi_depth(depth, loction, xLeftBottom, yLeftBottom, xRightTop - xLeftBottom, yRightTop - yLeftBottom, depth_img);
			dep_loc.depth = depth;
			dep_loc.locate = loction;
			cout << "locate: " << dep_loc.locate << endl;
			pub.publish(dep_loc);
		}
	}
	
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "track_01");
    track tra;

    // cv::startWindowThread();

    ros::spin();
    return 0;
}

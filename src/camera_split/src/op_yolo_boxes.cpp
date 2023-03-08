#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include "stdio.h"
#include <visualization_msgs/Marker.h>
#include <string>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
// #include "opencv01/yolo_box.h"
using namespace cv;
using namespace std;
using namespace cv;
using namespace cv::dnn;

ros::Publisher box_pub;
String modelConfiguration = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov2.cfg";
String modelBinary = "/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/yolov2.weights";
ifstream classNamesFile("/home/lzq/OpenCV_YOLO/yolov2-tiny-voc/coco.names");
// String modelConfiguration = "/home/yuan/darknet_ros_ws/src/darknet_ros/darknet/cfg/my-yolov3-tiny.cfg";
// String modelBinary = "/home/yuan/darknet_ros_ws/src/darknet_ros/darknet/mydata/weights/my-yolov3-tiny_20000.weights";
// ifstream classNamesFile("/home/yuan/darknet_ros_ws/src/darknet_ros/darknet/data/voc.names");
vector<string> classNamesVec;
float confidenceThreshold = 0.65;
dnn::Net net;
// vision::yolo_box yolo_box;
// darknet_ros_msgs::BoundingBoxes boxes;
// std::vector<darknet_ros_msgs::BoundingBox> boxes;
darknet_ros_msgs::BoundingBox  yolo_box;
// darknet_ros_msgs::BoundingBox boxes[10];
darknet_ros_msgs::BoundingBoxes  boxes;

void detect_callback(const sensor_msgs::ImageConstPtr msgImg)
{
    cv_bridge::CvImagePtr cvImgPtr;
    try{
        cvImgPtr = cv_bridge::toCvCopy(msgImg,sensor_msgs::image_encodings::BGR8);
        }
    catch(cv_bridge::Exception e)
        {
        ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
        return;
        }
    Mat frame=cvImgPtr->image;

    if (frame.empty())
			if (frame.channels() == 4)
				cvtColor(frame, frame, COLOR_BGRA2BGR);
		Mat inputBlob = blobFromImage(frame, 1 / 255.F, Size(416, 416), Scalar(), true, false);
		net.setInput(inputBlob, "data");
		Mat detectionMat = net.forward();
		vector<double> layersTimings;
		double freq = getTickFrequency() / 1000;
		double time = net.getPerfProfile(layersTimings) / freq;
		ostringstream ss;
		ss << "FPS: " << 1000 / time << " ; time: " << time << " ms";
		putText(frame, ss.str(), Point(20, 20), 0, 0.5, Scalar(0, 0, 255));

		for (int i = 0; i < detectionMat.rows; i++)
		{
			const int probability_index = 5;
			const int probability_size = detectionMat.cols - probability_index;
			float *prob_array_ptr = &detectionMat.at<float>(i, probability_index);
			size_t objectClass = max_element(prob_array_ptr, prob_array_ptr + probability_size) - prob_array_ptr;
            // cout<<"objectClass:"<<objectClass<<endl;
			float confidence = detectionMat.at<float>(i, (int)objectClass + probability_index);
				if (objectClass < classNamesVec.size())
				{
					ss.str("");
					ss << confidence;
                    yolo_box.probability=confidence;
                    // box.id = objectClass;
					String conf(ss.str());
					String label = String(classNamesVec[objectClass]) + ": " + conf;
					String label1 = String(classNamesVec[objectClass]);
                    // cout<<"label:"<<label1<<endl;
                    String name1="keyboard";
                    String name2="tvmonitor";
                    String name3="cup";

                if (confidence > confidenceThreshold&&(name1==label1||name2==label1||name3==label1))
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
                    rectangle(frame, object, Scalar(0, 255, 0));
                    yolo_box.xmin = xLeftBottom;
                    yolo_box.ymin = yLeftBottom;
                    yolo_box.xmax = xRightTop;
                    yolo_box.ymax = yRightTop;
                    yolo_box.Class = String(classNamesVec[objectClass]);
					int baseLine = 0;
					Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
					rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom),
						Size(labelSize.width, labelSize.height + baseLine)),
						Scalar(255, 255, 255), 1);
					putText(frame, label, Point(xLeftBottom, yLeftBottom + labelSize.height),
						FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
                    // boxes.push_back(box);
                    // boxes.bounding_boxes[objectClass] = box;
                    // [objectClass]=box;
                    // cout<<"33333333333"<<endl;
                    boxes.bounding_boxes.push_back(yolo_box);
                    // boxes.bounding_boxes.pop_back();
                    // box_pub.publish(yolo_box);
                    // cout<<"22222222222"<<endl;
                    cout<<"size:"<<boxes.bounding_boxes.size()<<endl;
				}
                // box_pub.publish(boxes);
			}
            // // boxes[i] = box;
            // cout<<"11111111111"<<endl;
            // boundings.bounding_boxes[i] = boxes[i];
            // cout<<"22222222222"<<endl;
		}
        if(detectionMat.rows)
        {
            boxes.header.stamp=ros::Time::now();
            box_pub.publish(boxes);
            boxes.bounding_boxes.clear();
        }
            
        // // boundings.bounding_boxes = boxes;
        // cout<<"**********"<<endl;
        // if (box.id)
        // {
        //     box_pub.publish(boundings);
        // }
		
		imshow("YOLOv3: Detections", frame);
		waitKey(30) ;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "frame_view");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    net = readNetFromDarknet(modelConfiguration, modelBinary);
    if (net.empty())
    {
        printf("Could not load net...\n");
        return 0;
    }

    
    
    if (classNamesFile.is_open())
    {
        string className = "";
        while (std::getline(classNamesFile, className))
            classNamesVec.push_back(className);
    }
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, detect_callback);
    // box_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("bounding_boxes", 10);
    box_pub = nh.advertise<darknet_ros_msgs::BoundingBoxes>("bounding_boxes", 1);
    ros::spin();

    return 0;
}
